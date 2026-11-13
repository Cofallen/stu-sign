import numpy as np

class RobotSensor:
    def __init__(self, model, data):
        """
        model: mujoco.MjModel
        data: mujoco.MjData
        """
        self.model = model
        self.data = data
        self.base_id = model.body('base_Link_del').id
        joint_names = dict(joint_l1_L='lf0_Joint', joint_l6_L='l20_Joint',
                           joint_wheel_L='l_wheel_Joint', joint_l1_R='rf0_Joint',
                           joint_l6_R='r20_Joint', joint_wheel_R='r_wheel_Joint')
        self.joint_qpos = {alias:int(model.joint(name).qposadr[0])
                           for alias,name in joint_names.items()}
        self.acc_sensor = data.sensor('base_lin_acc').data
        self.gyro_sensor = data.sensor('base_ang_vel').data

    # ---------------------------
    # IMU
    # ---------------------------
    def get_orientation_quat(self):
        """Return quaternion [w, x, y, z]"""
        # 当前 XML 的 framequat/body 对应惯性主轴，不能直接当机身 IMU 姿态。
        return self.data.xquat[self.base_id].copy()

    def get_acc(self):
        """Return linear acceleration (m/s^2)"""
        return self.acc_sensor.copy()

    def get_gyro(self):
        """Return angular velocity (rad/s)"""
        return self.gyro_sensor.copy()

    # ---------------------------
    # Joint positions
    # ---------------------------
    def get_joint_positions(self):
        """Return all joint positions as dict"""
        return {alias:self.data.qpos[index] for alias,index in self.joint_qpos.items()}

    # ---------------------------
    # Combined state (recommended)
    # ---------------------------
    def get_state(self):
        """Return full robot state"""
        quat = self.get_orientation_quat()
        return {
            "quat": quat,
            "acc": self.get_acc(),
            "gyro": self.get_gyro(),
            "joints": self.get_joint_positions(),
            "euler": self.quat_to_euler(quat),
        }

    # ---------------------------
    # Optional: quaternion → euler
    # ---------------------------
    def quat_to_euler(self, quat):
        """Convert quaternion [w,x,y,z] → roll, pitch, yaw"""
        w, x, y, z = quat

        # roll (x-axis)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

        # yaw (z-axis)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])