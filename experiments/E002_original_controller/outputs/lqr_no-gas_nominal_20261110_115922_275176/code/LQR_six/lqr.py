"""完整闭链的离线六状态 LQR 与腿长三次拟合。

get_k 在线性化工作点求 A/B/K；fit_ABK 拟合；GainSchedule 运行时仅算多项式。
质量、惯量、关节与闭链直接读取本项目 XML，不使用旧 chuan 等效杆参数。
状态 [theta,dtheta,s,dot_s,phi,dphi]；输入 [单轮力矩,单侧摆腿力矩]。
"""
import argparse
import sys
if not __package__:
    sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[1]))
    __package__ = 'LQR_six'
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.linalg import solve_continuous_are

ROOT = Path(__file__).resolve().parents[1]
from .parameters import LQR_FIT
Q = np.diag(LQR_FIT['q'])
R = np.diag(LQR_FIT['r'])


# K求解分成三步：完整模型线性化 → CARE → 多腿长拟合。
# 复杂的闭链Jacobian仅放lqr_model.py，不混在参数/矩阵操作中。
from .lqr_model import linearize, mass_parameters


def solve_lqr(A, B):
    """给定六状态A/B，用Q/R代价求u=-Kx。"""
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


def get_k(leg_length, qpos, reference_torque, model, epsilon=1e-5):
    A, B, projection_error = linearize(leg_length, qpos, reference_torque, model, epsilon)
    K = solve_lqr(A, B)
    return A, B, K, projection_error


def polyfit_matrix(x, matrices, order=3):
    """返回 [order+1,行,列]，系数按最高次到常数排列。"""
    return np.polyfit(x,np.asarray(matrices).reshape(len(x),-1),order).reshape(order+1,*np.asarray(matrices).shape[1:])


def fit_ABK(points, model, order=None):
    """1逐点求矩阵，2统一长度单位，3逐元素拟合并保存系数。"""
    order = LQR_FIT['order'] if order is None else order
    lengths = np.array([p['length'] for p in points])
    center, scale = LQR_FIT['center'], LQR_FIT['scale']  # 归一化避免米制三次项的病态拟合。
    samples = [get_k(p['length'],p['qpos'],p['ctrl'],model) for p in points]
    fits = {name:polyfit_matrix((lengths-center)/scale,[s[i] for s in samples],order).tolist()
            for i,name in enumerate(('A_coeffs','B_coeffs','K_coeffs'))}
    errors = [s[3] for s in samples]
    return dict(**fits,center_m=center,scale_m=scale,length_range_m=[LQR_FIT['minimum'], LQR_FIT['maximum']],order=order,
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
        self.coefficients = tuple(np.asarray(self.data[n],dtype=float)
                                  for n in ('A_coeffs','B_coeffs','K_coeffs'))

    def _argument(self, length):
        length = min(self.maximum,max(self.minimum,float(length)))
        return (length-self.data['center_m'])/self.data['scale_m'],length

    @staticmethod
    def _evaluate(coefficients, x):
        C3, C2, C1, C0 = coefficients
        return ((C3*x + C2)*x + C1)*x + C0

    def get_K(self, length):
        """控制循环只算K；不重复创建系数数组或计算不用的A/B。"""
        x,length = self._argument(length)
        return self._evaluate(self.coefficients[2],x),length

    def evaluate(self, length):
        x,length = self._argument(length)
        A = self._evaluate(self.coefficients[0], x)
        B = self._evaluate(self.coefficients[1], x)
        K = self._evaluate(self.coefficients[2], x)
        return A, B, K, length


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
