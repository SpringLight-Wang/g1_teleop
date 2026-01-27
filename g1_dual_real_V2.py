# Created by [ Wang chunguang - Henan University of Technology] 
import time
import sys
import threading
import numpy as np
import signal

try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmd_Type
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowState_Type
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmd_Default
except ImportError as e:
    print(f"❌ 导入错误: {e}")
    sys.exit(1)


try:
    from joycon_driver_dual import JoyConHandlerDual
    print("✅ 已导入 JoyCon 驱动")
except ImportError:
    print("❌ 错误: 找不到 joycon_driver_dual.py")
    sys.exit(1)

# ================= 配置参数 =================
NETWORK_INTERFACE = sys.argv[1] if len(sys.argv) > 1 else "eth0"
TOPIC_ARM_SDK = "rt/arm_sdk" 
TOPIC_LOWSTATE = "rt/lowstate"

DT = 0.02
JOINT_SPEED = 0.005
DEADZONE = 0.08
ARM_KP = 60.0    
ARM_KD = 1.5    
ARM_SDK_FLAG_INDEX = 29 

# [姿态映射参数]
ELBOW_SCALE = -1.5
ELBOW_OFFSET = 0.5
WRIST_OFFSET_L = 0.0 
WRIST_OFFSET_R = 0.0
WRIST_SCALE = 1.5
SH_YAW_SCALE = 0.8  

# [安全复位/锁定姿态]
RESET_POSE = {
    "sh_pitch": 0.0,
    "sh_roll": 0.0, 
    "sh_yaw": 0.0,
    "elbow": ELBOW_OFFSET, 
    "wrist_roll": 0.0
}

# [关节 ID 映射]
JOINT_MAP = {
    "left": {
        "sh_pitch": 15, "sh_roll": 16, "sh_yaw": 17, 
        "elbow": 18, "wrist_roll": 19 
    },
    "right": {
        "sh_pitch": 22, "sh_roll": 23, "sh_yaw": 24, 
        "elbow": 25, "wrist_roll": 26
    },
    "waist": {
        "yaw": 12, "roll": 13, "pitch": 14
    }
}

class G1Arm5HybridTeleop:
    def __init__(self):
        print(f">>> [Arm5 Hybrid] 初始化 (网卡: {NETWORK_INTERFACE})...")
        ChannelFactoryInitialize(0, NETWORK_INTERFACE)
        self.pub = ChannelPublisher(TOPIC_ARM_SDK, LowCmd_Type)
        self.pub.Init()
        self.sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_Type)
        self.sub.Init(self.state_handler)
        
        self.cmd = LowCmd_Default()
        
        for i in range(35): 
            if i < len(self.cmd.motor_cmd):
                self.cmd.motor_cmd[i].mode = 0x00
                self.cmd.motor_cmd[i].q = 0.0
                self.cmd.motor_cmd[i].kp = 0.0
                self.cmd.motor_cmd[i].kd = 0.0
                self.cmd.motor_cmd[i].tau = 0.0

        self.cmd.head = bytes([0xFE, 0xEF])
        self.cmd.level_flag = 0xFF
        self.cmd.gpio = 0
        self.cmd.sn = 0
        self.cmd.bandwidth = 0

        self.state = None 
        self.joy = JoyConHandlerDual()
        self.target_q = {} 
        self.running = False
        self.crc = CRC() 

        # === 锁定状态管理 ===
        self.reset_mode = {"left": False, "right": False}     # True: 锁定中, False: 控制中
        self.btn_last_state = {"left": 0, "right": 0}

        print(">>> 等待机器人状态数据...")
        wait_start = time.time()
        while self.state is None:
            time.sleep(0.1)
            if time.time() - wait_start > 10:
                print(f"❌ 超时：未收到数据。请检查网卡 {NETWORK_INTERFACE}")
                sys.exit(1)
        
        self.init_arm_pose()
        print("✅ 连接成功！")

    def state_handler(self, msg):
        self.state = msg

    def init_arm_pose(self):
        if self.state:
            for side in ["left", "right", "waist"]:
                for name, idx in JOINT_MAP[side].items():
                    if idx < len(self.state.motor_state):
                        self.target_q[idx] = self.state.motor_state[idx].q

    def update_joint(self, joint_idx, delta=0.0, val=None):
        if joint_idx not in self.target_q: return
        curr = self.target_q[joint_idx]
        if val is not None:
            self.target_q[joint_idx] = val
        else:
            self.target_q[joint_idx] = curr + delta

    def get_button_state(self, side):
        try:
            if side == "left" and self.joy.jc_l:
                return self.joy.jc_l.joycon.get_button_zl()
            elif side == "right" and self.joy.jc_r:
                return self.joy.jc_r.joycon.get_button_zr()
        except:
            pass
        return 0

    def process_joycon(self):
        inputs = self.joy.get_ik_states()
        if not inputs: return

        for side in ["left", "right"]:
            mapping = JOINT_MAP[side]
            
            # === 锁定/解锁 ===
            curr_btn = self.get_button_state(side)
            last_btn = self.btn_last_state[side]
            
            if curr_btn == 1 and last_btn == 0:
                self.reset_mode[side] = not self.reset_mode[side] # 切换状态
                mode_str = "🔒 锁定/复位" if self.reset_mode[side] else "🔓 解锁控制"
                print(f"\n>>> [{side.upper()}] {mode_str}")
            
            self.btn_last_state[side] = curr_btn 

            # === 锁定模式 ===
            if self.reset_mode[side]:
                for name, idx in mapping.items():
                    if name in RESET_POSE:
                        self.update_joint(idx, val=RESET_POSE[name])
                continue 

            # === 解锁模式 ===
            data = inputs[side]
            if not data: continue
            
            stick = data.get("stick", [0, 0])
            sx = stick[0] if abs(stick[0]) > DEADZONE else 0
            sy = stick[1] if abs(stick[1]) > DEADZONE else 0
            
            if sy and "sh_pitch" in mapping: self.update_joint(mapping["sh_pitch"], delta=-sy * JOINT_SPEED)
            if sx and "sh_roll" in mapping:  self.update_joint(mapping["sh_roll"], delta=-sx * JOINT_SPEED)

            if "rot" in data:
                rot = data["rot"]
                if "sh_yaw" in mapping:
                    self.update_joint(mapping["sh_yaw"], val=rot[2] * SH_YAW_SCALE)
                if "elbow" in mapping: 
                    self.update_joint(mapping["elbow"], val=ELBOW_OFFSET + rot[1] * ELBOW_SCALE)
                if "wrist_roll" in mapping:
                    s, o = (-WRIST_SCALE, WRIST_OFFSET_L) if side == "left" else (-WRIST_SCALE, WRIST_OFFSET_R)
                    self.update_joint(mapping["wrist_roll"], val=o + rot[0] * s)

    def run(self):
        self.running = True
        t = threading.Thread(target=self.print_loop)
        t.start()

        print("\n================ G1 Arm5 混合控制 ================")
        print("1. [站立] 请先用遥控器让机器人站立。")
        print("2. [控制] 按 ZL/ZR 键切换【锁定归位】和【手动控制】状态。")
        print("3. [开始] 按 Enter 键激活控制。")
        print("=============================================================")
        input("按 Enter 继续...")
        print("🚀 控制已激活！初始状态为：解锁")

        arm_indices = [idx for side in JOINT_MAP.values() for idx in side.values()]

        while self.running:
            start_t = time.time()
            self.cmd.sn += 1
            
            self.process_joycon()

            for i in range(35):
                if i >= len(self.cmd.motor_cmd): break
                m_cmd = self.cmd.motor_cmd[i]

                if i == ARM_SDK_FLAG_INDEX: 
                    m_cmd.q = 1.0 
                    m_cmd.kp = 0.0; m_cmd.kd = 0.0; m_cmd.tau = 0.0; m_cmd.dq = 0.0
                    continue

                if i in arm_indices:
                    m_cmd.mode = 0x01
                    m_cmd.q = self.target_q.get(i, 0.0)
                    m_cmd.dq = 0.0
                    m_cmd.tau = 0.0
                    m_cmd.kp = ARM_KP
                    m_cmd.kd = ARM_KD
                else:
                    m_cmd.mode = 0x00
                    m_cmd.q = 0.0; m_cmd.kp = 0.0; m_cmd.kd = 0.0; m_cmd.tau = 0.0

            self.cmd.crc = self.crc.Crc(self.cmd)
            self.pub.Write(self.cmd)

            sleep_t = DT - (time.time() - start_t)
            if sleep_t > 0:
                time.sleep(sleep_t)

    def stop(self):
        self.running = False
        print("\n⚠️ 正在退出...")
        for _ in range(5):
            if ARM_SDK_FLAG_INDEX < len(self.cmd.motor_cmd):
                self.cmd.motor_cmd[ARM_SDK_FLAG_INDEX].q = 0.0
                self.cmd.crc = self.crc.Crc(self.cmd)
                self.pub.Write(self.cmd)
            time.sleep(0.02)
        print("✅ 已释放控制权。")

    def print_loop(self):
        while self.running:
            if self.state:
                status_l = "LOCK" if self.reset_mode["left"] else "CTRL"
                status_r = "LOCK" if self.reset_mode["right"] else "CTRL"
                print(f"\r[L:{status_l} | R:{status_r}] SDK: ON", end="")
            time.sleep(0.2)

if __name__ == "__main__":
    teleop = G1Arm5HybridTeleop()
    def signal_handler(sig, frame):
        teleop.stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    try:
        teleop.run()
    except Exception as e:
        print(f"Error: {e}")
        teleop.stop()
