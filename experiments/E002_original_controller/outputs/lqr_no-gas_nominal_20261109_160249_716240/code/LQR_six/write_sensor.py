class RobotController:
    def __init__(self, model, data):
        self.model = model
        self.data = data

        # cache actuator ids (faster than string lookup every loop)
        self.actuator_ids = {
            "left_front": model.actuator("act_lf0").id,
            "left_back": model.actuator("act_lf20").id,
            "left_wheel": model.actuator("act_lw").id,
            "right_front": model.actuator("act_rf0").id,
            "right_back": model.actuator("act_rf20").id,
            "right_wheel": model.actuator("act_rw").id,
        }

    # ---------------------------
    # Low-level actuator control
    # ---------------------------
    def set_actuator(self, name, value):
        aid = self.actuator_ids[name]
        low, high = self.model.actuator_ctrlrange[aid]
        self.data.ctrl[aid] = max(low, min(high, float(value)))

    # XML 使用 motor：输入是 N·m，不是速度/位置目标。
    def stop_all(self):
        for name in self.actuator_ids:
            self.set_actuator(name, 0.0)
