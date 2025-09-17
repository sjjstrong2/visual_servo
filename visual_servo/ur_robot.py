import rtde_control
import rtde_receive
# from Robotiq_2F85.RobotiqGripper import RobotiqGripper
# from Camera.realsenseD435i import Camera

class UR_Robot:
    def __init__(self, tcp_host_ip="169.254.138.15", is_use_robotiq85=True, is_use_camera=True):
        """
        初始化RTDE连接。
        
        :param ip: 机器人的IP地址（str），例如 "169.254.138.15"。
        :param timeout: 连接超时时间（秒，int）。
        """
        self.ip = tcp_host_ip
        self.control = None
        self.receive = None
        try:
            self.control = rtde_control.RTDEControlInterface(self.ip,30004)
            self.receive = rtde_receive.RTDEReceiveInterface(self.ip,30004)
            print(f"已通过RTDE连接到机器人 {self.ip}")
        except Exception as e:
            raise ConnectionError(f"RTDE连接失败: {e}")
        if is_use_robotiq85:
            self.robotiq = RobotiqGripper(host=self.ip, verbose=False)
            self.robotiq.connect()
            self.robotiq.activate()
        if is_use_camera:
            self.camera = Camera()

    # 运动函数实现
    def movej(self, q, a=1.4, v=1.05, t=0, r=0):
        """
        关节空间线性移动（movej）。
        
        :param q: 关节位置列表（弧度，list of 6 floats）。
        :param a: 加速度（rad/s²，float）。
        :param v: 速度（rad/s，float）。
        :param t: 时间（秒，float）；如果>0，则优先于v和a。
        :param r: 交融半径（米，float）。
        """
        if t > 0:
            self.control.moveJ(q, velocity=v, acceleration=a, time=t, blend_radius=r)
        else:
            self.control.moveJ(q, velocity=v, acceleration=a, blend_radius=r)

    def movel(self, pose, a=1.2, v=0.25, r=0):
        """
        工艺移动（movep，圆形交融 + 线性）。
        
        :param pose: 目标姿态列表（[x, y, z, rx, ry, rz]，米和弧度，list of 6 floats）。
        :param a: 加速度（m/s²，float）。
        :param v: 速度（m/s，float）。
        :param r: 交融半径（米，float）。
        """
        self.control.moveL(pose, velocity=v, acceleration=a, blend_radius=r)

    def servoj(self, q, a=0, v=0, t=0.008, lookahead_time=0.1, gain=300):
        """
        关节空间伺服（servoj）。
        
        :param q: 关节位置列表（弧度，list of 6 floats）。
        :param a: 加速度（rad/s²，float）；当前版本中可能未使用。
        :param v: 速度（rad/s，float）；当前版本中可能未使用。
        :param t: 时间（秒，float）。
        :param lookahead_time: 前瞻时间（秒，float，范围0.03-0.2）。
        :param gain: 增益（float，范围100-2000）。
        """
        self.control.servoJ(q, velocity=v, acceleration=a, time=t, lookahead_time=lookahead_time, gain=gain)

    def speedj(self, qd, a=1.4, t_min=0.008):
        """
        关节速度控制（speedj）。
        
        :param qd: 关节速度列表（rad/s，list of 6 floats）。
        :param a: 加速度（rad/s²，float）。
        :param t_min: 最小时间（秒，float）。
        """
        self.control.speedJ(qd, acceleration=a, time=t_min)

    def speedl(self, xd, a=1.2, t_min=0.008):
        """
        工具速度控制（speedl）。
        
        :param xd: 工具速度列表（[vx, vy, vz, rx, ry, rz]，m/s和rad/s，list of 6 floats）。
        :param a: 加速度（m/s²，float）。
        :param t_min: 最小时间（秒，float）。
        """
        self.control.speedL(xd, acceleration=a, time=t_min)

    # 获取函数实现
    def get_actual_joint_positions(self):
        """
        获取实际关节位置（弧度）。
        
        :return: 关节位置列表（list of 6 floats）。
        """
        return self.receive.getActualQ()

    def get_actual_joint_speeds(self):
        """
        获取实际关节速度（rad/s）。
        
        :return: 关节速度列表（list of 6 floats）。
        """
        return self.receive.getActualQd()

    def get_actual_tcp_pose(self):
        """
        获取实际TCP姿态（[x, y, z, rx, ry, rz]）。
        
        :return: 姿态列表（list of 6 floats）。
        """
        return self.receive.getActualTCPPose()

    def get_actual_tcp_speed(self): 
        """
        获取实际TCP速度（[vx, vy, vz, rx, ry, rz]）。
        
        :return: 速度列表（list of 6 floats）。
        """
        return self.receive.getActualTCPSpeed()


    def get_tcp_force(self):
        """
        获取TCP力/力矩（[Fx, Fy, Fz, TRx, TRy, TRz]）。
        
        :return: 力/力矩列表（list of 6 floats）。
        """
        return self.receive.getActualTCPForce()
   
    def get_state(self, verbose=True):
        """
        获取机器人整体状态（关节 & TCP）。
        
        :param verbose: 是否打印状态信息（bool）
        :return: dict {
            "joints": {
                "positions": [...],
                "speeds": [...],
                "torques": [...]
            },
            "tcp": {
                "pose": [...],
                "speed": [...],
                "force": [...]
            }
        }
        """
        joints = {
            "positions": self.get_actual_joint_positions(),
            "speeds": self.get_actual_joint_speeds(),
        }

        tcp = {
            "pose": self.get_actual_tcp_pose(),
            "speed": self.get_actual_tcp_speed(),
            "force": self.get_tcp_force()
        }

        state = {"joints": joints, "tcp": tcp}

        if verbose:
            print("=== Joint State ===")
            print(f"Positions (rad): \n {joints['positions']}")
            print(f"Speeds (rad/s): \n {joints['speeds']}")
            print("=== TCP State ===")
            print(f"Pose [x,y,z,rx,ry,rz]: \n {tcp['pose']}")
            print(f"Speed [vx,vy,vz,rx,ry,rz]: \n {tcp['speed']}")
            print(f"Force [Fx,Fy,Fz,TRx,TRy,TRz]: \n {tcp['force']}")
            print("====================")

        return state

    def close(self):
        """
        关闭RTDE连接。
        """
        if self.control:
            self.control.disconnect()
        if self.receive:
            self.receive.disconnect()
        if hasattr(self, 'robotiq') and self.robotiq:
            self.robotiq.disconnect()
        print("RTDE连接已关闭")

# 示例使用
if __name__ == "__main__":
    robot = UR_Robot("169.254.138.15", is_use_robotiq85= False, is_use_camera= False)  # 替换为实际IP
    try:
        # 运动示例
        robot.get_state()
    except Exception as e:
        print(f"错误: {e}")
    finally:
        robot.close()