"""完整闭链的离线六状态 LQR 与腿长三次拟合。

get_k 在线性化工作点求 A/B/K；fit_ABK 拟合；GainSchedule 运行时仅算多项式。
质量、惯量、关节与闭链直接读取本项目 XML，不使用旧 chuan 等效杆参数。
状态 [theta,dtheta,s,dot_s,phi,dphi]；输入 [单轮力矩,单侧摆腿力矩]。
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.linalg import solve_continuous_are

ROOT = Path(__file__).resolve().parents[1]
Q = np.diag([3000., 1., 500., 1., 30000., 1.])
R = np.eye(2)


def mass_parameters(model):
    """kg；左右各使用半个机身+该侧真实独立连杆，轮子单列，禁止重复计重。"""
    body = float(model.body_mass[model.body('base_Link_del').id])
    legs, wheels = {}, {}
    for side in ('l', 'r'):
        names = [side+n for n in ('f0_Link', 'f1_Link', '20_Link', '21_Link', '22_Link', '23_Link')]
        legs[side] = float(sum(model.body_mass[model.body(n).id] for n in names))
        wheels[side] = float(model.body_mass[model.body(side+'_wheel_Link').id])
    total = float(sum(model.body_mass))
    if not np.isclose(body+sum(legs.values())+sum(wheels.values()), total):
        raise ValueError('模型质量分类不完整，先核对新增连杆。')
    return dict(body_total_kg=body, body_per_side_kg=body/2, leg_kg=legs,
                wheel_kg=wheels, total_kg=total, wheel_radius_m=.06)


def get_k(leg_length, qpos, reference_torque, model, epsilon=1e-5):
    """固定腿长的平面切空间线性化，保留完整模型惯量与约束。

    qpos/torque 应来自该腿长稳定站立工作点；不是按比例缩放 nominal K。
    与原标定相同，干摩擦在求导时暂置零以避免零速度不可微；仿真不改摩擦。
    """
    data = mujoco.MjData(model)
    qref = np.asarray(qpos).copy()
    data.qpos[:] = qref
    mujoco.mj_forward(model, data)
    nv = model.nv
    active = [int(model.joint(n).dofadr[0]) for n in ('lf0_Joint','l20_Joint','rf0_Joint','r20_Joint')]
    wheel_dofs = [int(model.joint(s+'_wheel_Joint').dofadr[0]) for s in ('l','r')]
    passive = [i for i in range(6, nv) if i not in active+wheel_dofs]
    base = model.body('base_Link_del').id

    def geometry():
        equality = np.asarray(data.efc_J).reshape(-1, nv)
        equality = equality[data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY]
        coupling = np.zeros((nv, 4)); coupling[active] = np.eye(4)
        coupling[passive] = -np.linalg.pinv(equality[:,passive], rcond=1e-5) @ equality[:,active]
        lengths, angles, jacobians, length_rows, theta_rows, contacts = [], [], [], [], [], []
        for i, side in enumerate(('l','r')):
            wheel = model.body(side+'_wheel_Link').id
            hip = data.xanchor[model.joint(side+'f0_Joint').id]
            jp, jr, hj = np.zeros((3,nv)), np.zeros((3,nv)), np.zeros((3,nv))
            mujoco.mj_jacBody(model,data,jp,jr,wheel)
            mujoco.mj_jac(model,data,hj,np.zeros_like(jr),hip,base)
            relative = data.xpos[wheel]-hip
            length = np.linalg.norm(relative)
            dl = relative@(jp-hj)/length
            theta = (relative[2]*(jp-hj)[0]-relative[0]*(jp-hj)[2])/(relative[0]**2+relative[2]**2)
            lengths.append(length); angles.append(np.arctan2(-relative[0],-relative[2]))
            length_rows.append(dl); theta_rows.append(theta)
            jacobians.append(np.vstack([dl,theta])@coupling[:,2*i:2*i+2])
            # 轮底接触点速度 Jacobian：轮心 + omega cross (0,0,-R)。
            skew = np.array([[0.,.06,0.],[-.06,0.,0.],[0.,0.,0.]])
            contacts.append(jp-skew@jr)
        C = np.vstack([np.mean(theta_rows,axis=0),np.eye(nv)[0],-np.eye(nv)[4]])
        return np.array(lengths),np.array(angles),jacobians,C,equality,contacts,length_rows

    lengths, angles, jac, C, equality, contacts, length_rows = geometry()
    if max(abs(lengths-leg_length)) > .003:
        raise ValueError('工作点的实际腿长与标注不符。')
    constraint = np.vstack([equality,*contacts,np.eye(nv)[3],np.eye(nv)[5],*length_rows])
    equations = np.vstack([constraint,C])
    target = np.vstack([np.zeros((len(constraint),3)),np.eye(3)])
    tangent = np.linalg.lstsq(equations,target,rcond=1e-5)[0]
    tangent_error = float(np.max(abs(equations@tangent-target)))
    # 真实机构左右略不对称，六状态平面投影不是全机构的精确坐标变换。
    # 记录残差并以非线性闭环复验；超过1%则拒绝拟合。
    if tangent_error > 1e-2:
        raise ValueError(f'切空间残差过大：{tangent_error}')
    bias = [np.linalg.solve(jac[i].T,np.asarray(reference_torque)[[0,4] if i==0 else [2,5]]) for i in range(2)]

    def acceleration(position, velocity, u):
        data.qpos[:] = qref
        mujoco.mj_integratePos(model,data.qpos,tangent@position,1.)
        data.qvel[:] = tangent@velocity
        mujoco.mj_forward(model,data)
        L, th, D, _, _, _, dl = geometry()
        data.ctrl[:] = 0.
        for i in range(2):
            # 支撑/滚转/双腿同步内环与现有标定一致；LQR两输入保持左右对称。
            roll = 2*np.arctan2(data.qpos[4],data.qpos[3])
            delta_force = (-1000*roll-80*data.qvel[3])*(1 if i==0 else -1)
            force = bias[i][0]+2000*(lengths[i]-L[i])-80*(dl[i]@data.qvel)+delta_force
            delta = th[0]-th[1]
            delta_dot = D[0][1]@data.qvel[active[:2]]-D[1][1]@data.qvel[active[2:]]
            correction = (-50*delta-3*delta_dot)*(1 if i==0 else -1)
            data.ctrl[[0,4] if i==0 else [2,5]] = D[i].T@np.array([force,bias[i][1]+u[1]+correction])
        data.ctrl[1], data.ctrl[3] = u[0], -u[0]
        mujoco.mj_forward(model,data)
        return C@data.qacc

    friction = model.dof_frictionloss.copy()
    try:
        model.dof_frictionloss[:] = 0.
        A, B = np.zeros((6,6)), np.zeros((6,2))
        A[[0,2,4],[1,3,5]] = 1.
        eps = epsilon
        for j in range(3):
            v = np.eye(3)[j]*eps
            A[[1,3,5],2*j] = (acceleration(v,np.zeros(3),[0,0])-acceleration(-v,np.zeros(3),[0,0]))/(2*eps)
            A[[1,3,5],2*j+1] = (acceleration(np.zeros(3),v,[0,0])-acceleration(np.zeros(3),-v,[0,0]))/(2*eps)
        for j in range(2):
            u = np.eye(2)[j]*eps
            B[[1,3,5],j] = (acceleration(np.zeros(3),np.zeros(3),u)-acceleration(np.zeros(3),np.zeros(3),-u))/(2*eps)
        P = solve_continuous_are(A,B,Q,R)
        K = np.linalg.solve(R,B.T@P)
    finally:
        model.dof_frictionloss[:] = friction
    return A,B,K,tangent_error


def polyfit_matrix(x, matrices, order=3):
    """返回 [order+1,行,列]，系数按最高次到常数排列。"""
    return np.polyfit(x,np.asarray(matrices).reshape(len(x),-1),order).reshape(order+1,*np.asarray(matrices).shape[1:])


def fit_ABK(points, model, order=3):
    lengths = np.array([p['length'] for p in points])
    center, scale = .26, .08  # 归一化避免米制三次项的病态拟合。
    samples = [get_k(p['length'],p['qpos'],p['ctrl'],model) for p in points]
    fits = {name:polyfit_matrix((lengths-center)/scale,[s[i] for s in samples],order).tolist()
            for i,name in enumerate(('A_coeffs','B_coeffs','K_coeffs'))}
    errors = [s[3] for s in samples]
    return dict(**fits,center_m=center,scale_m=scale,length_range_m=[.18,.34],order=order,
                masses=mass_parameters(model),Q=Q.tolist(),R=R.tolist(),
                source='Full closed-chain model planar numerical linearization; per-side virtual torque inputs',
                lengths_m=lengths.tolist(),tangent_errors=errors,
                samples={n:[s[i].tolist() for s in samples] for i,n in enumerate(('A','B','K'))})


class GainSchedule:
    def __init__(self, variant='no-gas'):
        path = ROOT/'gain_schedule.json'
        self.config = json.loads(path.read_text())
        self.data = self.config['variants'][variant]
        filename = 'robot_no_gas_spring.xml' if variant == 'no-gas' else 'robot_gas_spring_damped.xml'
        actual = hashlib.sha256((ROOT/'assets'/filename).read_bytes()).hexdigest()
        if self.data['model_sha256'] != actual:
            raise ValueError('XML与拟合增益版本不符，请重新标定。')
        self.minimum,self.maximum = self.data['length_range_m']

    def evaluate(self, length):
        length = float(np.clip(length,self.minimum,self.maximum))
        x = (length-self.data['center_m'])/self.data['scale_m']
        result = []
        for name in ('A_coeffs','B_coeffs','K_coeffs'):
            coefficients = np.array(self.data[name])
            value = coefficients[0].copy()
            for coefficient in coefficients[1:]:
                value = value*x+coefficient
            result.append(value)
        return (*result,length)


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['no-gas','gas'], default='no-gas')
    parser.add_argument('--points',type=Path,required=True,help='稳定工作点JSON：length/qpos/ctrl')
    parser.add_argument('--output',type=Path,required=True,help='写新路径，不覆写历史拟合')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('输出已存在，请使用新路径。')
    points = json.loads(args.points.read_text())
    variants = {}
    for variant in [args.variant]:
        filename = 'robot_no_gas_spring.xml' if variant == 'no-gas' else 'robot_gas_spring_damped.xml'
        model = mujoco.MjModel.from_xml_path(str(ROOT/'assets'/filename))
        variants[variant] = fit_ABK(points,model)
        variants[variant]['model_sha256'] = hashlib.sha256((ROOT/'assets'/filename).read_bytes()).hexdigest()
    args.output.write_text(json.dumps(dict(variants=variants,points_sha256=hashlib.sha256(args.points.read_bytes()).hexdigest()),indent=2)+'\n')
    print(args.output)
