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

# G1 Arm5 版本的 Enable 标志位
ARM_SDK_FLAG_INDEX = 29 

# 姿态映射
ELBOW_SCALE = -1.5
ELBOW_OFFSET = 0.5
WRIST_OFFSET_L = 0.0 
WRIST_OFFSET_R = 0.0
WRIST_SCALE = 1.5

# 关节 ID 映射 - G1 Arm5
JOINT_MAP = {
    "left": {
        "sh_pitch": 15, "sh_roll": 16, "sh_yaw": 17,
        "elbow": 18, 
        "wrist_roll": 19 
    },
    "right": {
        "sh_pitch": 22, "sh_roll": 23, "sh_yaw": 24,
        "elbow": 25, 
        "wrist_roll": 26
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
        """读取当前手臂角度作为初始目标"""
        if self.state:
            for side in ["left", "right"]:
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

    def process_joycon(self):
        inputs = self.joy.get_ik_states()
        if not inputs: return

        for side in ["left", "right"]:
            data = inputs[side]
            if not data: continue
            mapping = JOINT_MAP[side]
            
            stick = data.get("stick", [0, 0])
            sx = stick[0] if abs(stick[0]) > DEADZONE else 0
            sy = stick[1] if abs(stick[1]) > DEADZONE else 0
            
            if sy and "sh_pitch" in mapping: self.update_joint(mapping["sh_pitch"], delta=-sy * JOINT_SPEED)
            if sx and "sh_roll" in mapping:  self.update_joint(mapping["sh_roll"], delta=-sx * JOINT_SPEED)

            if "rot" in data:
                rot = data["rot"]
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
        print("1. 请先用遥控器让机器人【站立】。")
        print("2. 按 Enter 键开始，JoyCon 将接管手臂，遥控器继续控制走路。")
        print("==================================================")
        input("按 Enter 继续...")
        print("🚀 控制已激活！")

        arm_indices = [idx for side in JOINT_MAP.values() for idx in side.values()]

        while self.running:
            start_t = time.time()
            self.cmd.sn += 1
            
            self.process_joycon()

            for i in range(35):
                if i >= len(self.cmd.motor_cmd): break
                m_cmd = self.cmd.motor_cmd[i]

                # 开启 Arm SDK 模式
                if i == ARM_SDK_FLAG_INDEX: 
                    m_cmd.q = 1.0  # Enable
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
                self.cmd.motor_cmd[ARM_SDK_FLAG_INDEX].q = 0.0 # Disable
                self.cmd.crc = self.crc.Crc(self.cmd)
                self.pub.Write(self.cmd)
            time.sleep(0.02)
        print("✅ 已释放控制权。")

    def print_loop(self):
        while self.running:
            if self.state:
                idx = 18
                if idx < len(self.state.motor_state):
                    q = self.state.motor_state[idx].q
                    t = self.target_q.get(idx, 0.0)
                    print(f"\r[Arm5] L-Elbow Real:{q:.2f} Tar:{t:.2f} | SDK: ON", end="")
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
