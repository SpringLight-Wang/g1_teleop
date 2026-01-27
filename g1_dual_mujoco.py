# Created by [ Wang chunguang - Henan University of Technology] 
import time
import sys
import os
import numpy as np
import mujoco
import mujoco.viewer

# [配置] 请修改为本机实际 XML 路径
MODEL_PATH = "/home/wcg/g1_teleop_project/unitree_description/mjcf/g1_liao.xml"

# [参数]
JOINT_SPEED = 0.01 
POS_KP = 500.0  
WRIST_KP = 150.0
DEADZONE = 0.08  

# [姿态映射]
ELBOW_SCALE = -1.5
ELBOW_OFFSET = 0.5
WRIST_OFFSET_L = -1.57
WRIST_OFFSET_R = 1.57
WRIST_SCALE = 1.5
SH_YAW_SCALE = 0.8 

# [初始姿态 / 复位姿态]
INIT_POSE = {
    "left_shoulder_pitch_joint": 0.0, "left_shoulder_roll_joint": 0.2, "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 1.57, "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.0, "right_shoulder_roll_joint": -0.2, "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 1.57, "right_wrist_roll_joint": 0.0,
}

try:
    from joycon_driver_dual import JoyConHandlerDual
except ImportError:
    print("!!! 错误: 找不到手柄驱动 joycon_driver_dual.py")
    sys.exit(1)


class MujocoDualArmTeleop:
    def __init__(self, model_path):
        if not os.path.exists(model_path):
            print(f"❌ 模型文件不存在: {model_path}\n请修改代码中的 MODEL_PATH")
            sys.exit(1)
        self.m = mujoco.MjModel.from_xml_path(model_path)
        self.d = mujoco.MjData(self.m)
        print(">>> 正在连接手柄... 请保持手柄静止以进行校准！")
        self.joy = JoyConHandlerDual()
        self.act_info = {}
        self.arm_joints = {"left": {}, "right": {}}
        
        # === 状态锁 ===
        self.reset_mode = {"left": False, "right": False}
        self.btn_last_state = {"left": 0, "right": 0}

        # 关节映射
        joint_name_map = {}
        for i in range(self.m.nu):
            act_name = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if not act_name: continue
            trnid = self.m.actuator_trnid[i, 0]
            self.act_info[i] = {
                "name": act_name, "qadr": self.m.jnt_qposadr[trnid],
                "dofadr": self.m.jnt_dofadr[trnid], "limits": self.m.jnt_range[trnid]
            }
            joint_name_map[act_name] = i
            side = "left" if "left" in act_name else "right" if "right" in act_name else None
            
            if side:
                if "shoulder_pitch" in act_name: self.arm_joints[side]["sh_pitch"] = i
                if "shoulder_roll" in act_name:  self.arm_joints[side]["sh_roll"] = i
                if "shoulder_yaw" in act_name:   self.arm_joints[side]["sh_yaw"] = i
                if "elbow" in act_name:          self.arm_joints[side]["elbow"] = i
                if "wrist" in act_name and "roll" in act_name: self.arm_joints[side]["wrist"] = i

        mujoco.mj_resetData(self.m, self.d)
        # 应用初始姿态
        for j_name, j_val in INIT_POSE.items():
            if j_name in joint_name_map:
                self.d.qpos[self.act_info[joint_name_map[j_name]]["dofadr"]] = j_val
            else:
                for k in joint_name_map:
                    if j_name in k:
                        self.d.qpos[self.act_info[joint_name_map[k]]["dofadr"]] = j_val
                        break

        mujoco.mj_forward(self.m, self.d)
        self.q_target = self.d.qpos.copy()

    def update_target(self, act_id, delta=0.0, val=None):
        if act_id == -1: return
        idx = self.act_info[act_id]["dofadr"]
        lim = self.act_info[act_id]["limits"]
        self.q_target[idx] = np.clip(val if val else self.q_target[idx] + delta, lim[0], lim[1])

    def get_button_state(self, side):
        try:
            if side == "left" and self.joy.jc_l:
                return self.joy.jc_l.joycon.get_button_zl()
            elif side == "right" and self.joy.jc_r:
                return self.joy.jc_r.joycon.get_button_zr()
        except:
            pass
        return 0

    def process_side(self, side, data):
        j = self.arm_joints[side]
        
        # === 按键切换逻辑 ===
        curr_btn = self.get_button_state(side)
        last_btn = self.btn_last_state[side]
        
        if curr_btn == 1 and last_btn == 0:
            self.reset_mode[side] = not self.reset_mode[side]
            print(f">>> {side.upper()} Mode: {'LOCKED/RESET' if self.reset_mode[side] else 'CONTROL'}")

        self.btn_last_state[side] = curr_btn

        # === 锁定模式 ===
        if self.reset_mode[side]:
            # 持续强制复位
            for name, val in INIT_POSE.items():
                if side in name:
                    target_act_id = -1
                    if "shoulder_pitch" in name: target_act_id = j.get("sh_pitch")
                    elif "shoulder_roll" in name: target_act_id = j.get("sh_roll")
                    elif "shoulder_yaw" in name: target_act_id = j.get("sh_yaw")
                    elif "elbow" in name: target_act_id = j.get("elbow")
                    elif "wrist_roll" in name: target_act_id = j.get("wrist")
                    
                    if target_act_id and target_act_id != -1:
                        self.update_target(target_act_id, val=val)
            return

        # === 控制模式 ===
        if not data: return
        sx = data["stick"][0] if abs(data["stick"][0]) > DEADZONE else 0
        sy = data["stick"][1] if abs(data["stick"][1]) > DEADZONE else 0

        if sy and j.get("sh_pitch", -1) != -1: self.update_target(j["sh_pitch"], delta=-sy * JOINT_SPEED)
        if sx and j.get("sh_roll", -1) != -1: self.update_target(j["sh_roll"], delta=-sx * JOINT_SPEED)
        
        if "rot" in data:
            if j.get("sh_yaw", -1) != -1:
                self.update_target(j["sh_yaw"], val=data["rot"][2] * SH_YAW_SCALE)
            if j.get("elbow", -1) != -1: 
                self.update_target(j["elbow"], val=ELBOW_OFFSET + data["rot"][1] * ELBOW_SCALE)
            if j.get("wrist", -1) != -1:
                s, o = (-WRIST_SCALE, WRIST_OFFSET_L) if side == "left" else (-WRIST_SCALE, WRIST_OFFSET_R)
                self.update_target(j["wrist"], val=o + data["rot"][0] * s)

    def run(self):
        print("按 ZL(左)/ZR(右) 切换 复位锁定 状态")
        with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
            while viewer.is_running():
                step_start = time.time()
                inputs = self.joy.get_ik_states()
                if inputs:
                    self.process_side("left", inputs["left"])
                    self.process_side("right", inputs["right"])

                for i, info in self.act_info.items():
                    kp = POS_KP
                    if "wrist" in info["name"]:
                        kp = WRIST_KP
                    elif "elbow" in info["name"] or "shoulder" in info["name"]:
                        kp = 200.0
                    
                    self.d.ctrl[i] = kp * (self.q_target[info["dofadr"]] - self.d.qpos[info["qadr"]]) - 10.0 * \
                                     self.d.qvel[info["dofadr"]] + self.d.qfrc_bias[info["dofadr"]]

                mujoco.mj_step(self.m, self.d)
                viewer.sync()
                if (t := self.m.opt.timestep - (time.time() - step_start)) > 0: time.sleep(t)


if __name__ == "__main__":
    MujocoDualArmTeleop(MODEL_PATH).run()
