# Created by [ Wang chunguang - Henan University of Technology] 

import sys
import time
import numpy as np

print(">>> 正在尝试导入 joyconrobotics...")
from joyconrobotics import JoyconRobotics
print("✅ 导入成功！")



class JoyConHandlerDual:
    def __init__(self):
        self.jc_l = None;
        self.jc_r = None
        self.calib = {}
        try:
            self.jc_l = JoyconRobotics("left"); print("✅ Left connected")
        except:
            pass
        try:
            self.jc_r = JoyconRobotics("right"); print("✅ Right connected")
        except:
            pass

    def _norm(self, val, dev, axis):
        if val is None: return 0.0
        if dev not in self.calib: self.calib[dev] = {}
        if axis not in self.calib[dev]: self.calib[dev][axis] = val
        return float(np.clip((val - self.calib[dev][axis]) / 1200.0, -1.0, 1.0))

    def _read(self, dev, did):
        if not dev: return None
        sx, sy, rot = 0.0, 0.0, np.zeros(3)
        if hasattr(dev, 'joycon'):  # joyconrobotics
            jc = dev.joycon
            is_r = jc.is_right() if hasattr(jc, 'is_right') else False
            try:
                sy = self._norm(jc.get_stick_right_vertical() if is_r else jc.get_stick_left_vertical(), did, 'y')
            except:
                pass
            try:
                sx = self._norm(jc.get_stick_right_horizontal() if is_r else jc.get_stick_left_horizontal(), did, 'x')
            except:
                pass
        elif hasattr(dev, 'read_input'):  # Raw
            d = dev.read_input()
            if d: sx, sy = d['stick']

        if hasattr(dev, 'get_control'):
            try:
                r = dev.get_control()
                if len(r) > 0 and len(r[0]) >= 6: rot = np.array([r[0][3], r[0][4], r[0][5]])
            except:
                pass
        return {"stick": np.array([sx, sy]), "rot": rot}

    def get_ik_states(self):
        return {
            "left": self._read(self.jc_l, "L"),
            "right": self._read(self.jc_r, "R")
        }

