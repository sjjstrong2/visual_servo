import socket
import time
from enum import IntEnum, Enum
from typing import Optional

# -----------------------------
# 枚举定义：对应 C++ 版本中的 enum
# -----------------------------

class ConnectionState(Enum):
    """连接状态"""
    DISCONNECTED = 0   # 未连接
    CONNECTED = 1      # 已连接

class eStatus(IntEnum):
    """Gripper 的整体状态 (STA)"""
    RESET = 0          # 未激活
    ACTIVATING = 1     # 激活中
    ACTIVE = 3         # 已激活

class eObjectStatus(IntEnum):
    """物体检测状态 (OBJ)"""
    MOVING = 0                  # Gripper 正在移动
    STOPPED_OUTER_OBJECT = 1    # 遇到外部物体停止（在张开时）
    STOPPED_INNER_OBJECT = 2    # 遇到内部物体停止（在夹紧时）
    AT_DEST = 3                 # 到达目标位置

class eMoveMode(Enum):
    """移动模式"""
    START_MOVE = 0    # 仅发送移动命令，不等待结果
    WAIT_FINISHED = 1 # 发送命令并阻塞，直到动作完成

class eUnit(Enum):
    """参数单位（仅占位，与 C++ 一致）"""
    UNIT_DEVICE = 0
    UNIT_NORMALIZED = 1
    UNIT_PERCENT = 2
    UNIT_MM = 3

class eMoveParameter(Enum):
    """移动参数（用于 get/set）"""
    POSITION = 0
    SPEED = 1
    FORCE = 2

class ePostionId(Enum):
    """预定义位置"""
    OPEN = 0     # 打开位置
    CLOSE = 1    # 关闭位置

class eFaultCode(IntEnum):
    """故障代码 (FLT)"""
    NO_FAULT = 0
    FAULT_ACTION_DELAYED = 1
    FAULT_ACTIVATION_BIT = 2
    FAULT_TEMPERATURE = 3
    FAULT_COMM = 4
    FAULT_UNDER_VOLTAGE = 5
    FAULT_EMCY_RELEASE_ACTIVE = 6
    FAULT_INTERNAL = 7
    FAULT_ACTIVATION = 8
    FAULT_OVERCURRENT = 9
    FAULT_EMCY_RELEASE_FINISHED = 10


# -----------------------------
# 主类：RobotiqGripper
# -----------------------------

class RobotiqGripper:
    """
    Python 版本的 Robotiq Gripper 控制类
    对应 ur_rtde C++ API 的 RobotiqGripper 类。
    
    通过 TCP Socket 与 URCap Gripper 服务器通信，支持完整的 gripper 控制与状态读取。
    """

    DEFAULT_PORT = 63352  # Robotiq_Grippers URCap 默认端口

    def __init__(self, host: str, port: int = DEFAULT_PORT, verbose: bool = False):
        """
        初始化 gripper 控制类
        :param host: UR 机器人 IP 地址
        :param port: 通信端口（默认 63352）
        :param verbose: 是否打印通信日志
        """
        self.host = host
        self.port = port
        self.verbose = verbose
        self.sock: Optional[socket.socket] = None
        self.state = ConnectionState.DISCONNECTED

    # -----------------------------
    # 连接相关方法
    # -----------------------------
    def connect(self, timeout_ms: int = 2000):
        """
        建立与 Gripper URCap 的 TCP 连接
        :param timeout_ms: 超时时间 (毫秒)
        """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout_ms / 1000.0)
        self.sock.connect((self.host, self.port))
        self.state = ConnectionState.CONNECTED
        if self.verbose:
            print(f"[Gripper] Connected to {self.host}:{self.port}")

    def disconnect(self):
        """
        断开与 Gripper 的连接
        """
        if self.sock:
            self.sock.close()
        self.sock = None
        self.state = ConnectionState.DISCONNECTED
        if self.verbose:
            print(f"[Gripper] Disconnected")

    def isConnected(self) -> bool:
        """
        查询是否已连接
        :return: True = 已连接, False = 未连接
        """
        return self.state == ConnectionState.CONNECTED

    def _send(self, cmd: str) -> str:
        """
        内部方法：发送命令并获取返回值
        :param cmd: ASCII 命令（如 "SET POS 255"）
        :return: 响应字符串
        """
        if not self.sock:
            raise RuntimeError("Not connected to gripper")
        full = (cmd.strip() + "\n").encode('ascii')
        if self.verbose:
            print(f">>> {cmd}")
        self.sock.sendall(full)
        data = self.sock.recv(1024)
        resp = data.strip().decode('ascii')
        if self.verbose:
            print(f"<<< {resp}")
        return resp

    # -----------------------------
    # 激活/校准
    # -----------------------------
    def activate(self, auto_calibrate: bool = False):
        """
        激活 gripper
        :param auto_calibrate: 是否自动校准（先关后开）
        """
        resp = self._send("SET ACT 0")
        resp = self._send("SET ACT 1")
        if auto_calibrate:
            self.autoCalibrate()
        return resp

    def autoCalibrate(self, Speed: float = -1.0):
        """
        自动校准 gripper
        :param Speed: 校准速度百分比 (0-100)，默认 -1 使用设备默认值
        """
        if Speed >= 0:
            val = int(Speed / 100.0 * 255)
            self._send(f"SET SPE {val}")
        # 执行一次夹紧动作来完成校准
        self._send("SET GTO 1")
        return self._send("GET OBJ")

    def isActive(self) -> bool:
        """
        判断 gripper 是否处于激活状态
        :return: True = 已激活
        """
        resp = self._send("GET STA")
        try:
            status = int(resp.split()[-1])
            return status == eStatus.ACTIVE
        except:
            return False

    # -----------------------------
    # 位置信息
    # -----------------------------
    def getOpenPosition(self) -> float:
        """获取 gripper 打开位置（设备单位，约为 0）"""
        return 0.0

    def getClosedPosition(self) -> float:
        """获取 gripper 关闭位置（设备单位，约为 255）"""
        return 255.0

    def getCurrentPosition(self) -> float:
        """
        获取当前 gripper 实际位置
        :return: PRE 寄存器值 (0-255)
        """
        resp = self._send("GET PRE")
        return float(resp.split()[-1])

    def isOpen(self) -> bool:
        """判断 gripper 是否处于完全打开位置"""
        return self.getCurrentPosition() == self.getOpenPosition()

    def isClosed(self) -> bool:
        """判断 gripper 是否处于完全关闭位置"""
        return self.getCurrentPosition() == self.getClosedPosition()

    # -----------------------------
    # 动作控制
    # -----------------------------
    def move(self, Position: float, Speed: float = -1.0, Force: float = -1.0,
             MoveMode: eMoveMode = eMoveMode.START_MOVE) -> eObjectStatus:
        """
        控制 gripper 移动到指定位置
        :param Position: 目标位置 (0-255)
        :param Speed: 移动速度百分比 (0-100)，默认 -1 表示保持不变
        :param Force: 夹持力百分比 (0-100)，默认 -1 表示保持不变
        :param MoveMode: 是否等待动作完成
        :return: 最终的物体检测状态 (OBJ)
        """
        if Speed >= 0:
            self._send(f"SET SPE {int(Speed / 100.0 * 255)}")
        if Force >= 0:
            self._send(f"SET FOR {int(Force / 100.0 * 255)}")
        self._send(f"SET POS {int(Position)}")
        self._send("SET GTO 1")
        obj = int(self._send("GET OBJ").split()[-1])
        if MoveMode == eMoveMode.WAIT_FINISHED:
            while obj == eObjectStatus.MOVING:
                time.sleep(0.05)
                obj = int(self._send("GET OBJ").split()[-1])
        return eObjectStatus(obj)

    def open(self, Speed: float = -1.0, Force: float = -1.0,
             MoveMode: eMoveMode = eMoveMode.START_MOVE) -> eObjectStatus:
        """张开 gripper"""
        return self.move(self.getOpenPosition(), Speed, Force, MoveMode)

    def close(self, Speed: float = -1.0, Force: float = -1.0,
              MoveMode: eMoveMode = eMoveMode.START_MOVE) -> eObjectStatus:
        """闭合 gripper"""
        return self.move(self.getClosedPosition(), Speed, Force, MoveMode)

    def emergencyRelease(self, Direction: ePostionId,
                         MoveMode: eMoveMode = eMoveMode.WAIT_FINISHED):
        """
        执行紧急释放动作
        :param Direction: 释放方向 (OPEN / CLOSE)
        :param MoveMode: 是否等待完成
        """
        self._send("SET ATR 1")
        self._send(f"SET ARD {1 if Direction == ePostionId.OPEN else 0}")
        if MoveMode == eMoveMode.WAIT_FINISHED:
            while True:
                obj = int(self._send("GET OBJ").split()[-1])
                if obj != eObjectStatus.MOVING:
                    break

    # -----------------------------
    # 状态/故障
    # -----------------------------
    def getFault(self) -> eFaultCode:
        """
        获取 gripper 故障代码
        :return: eFaultCode 枚举
        """
        resp = self._send("GET FLT")
        code = int(resp.split()[-1])
        return eFaultCode(code)

    # -----------------------------
    # 夹爪状态
    # -----------------------------

    def get_state(self, verbose: bool = True) -> dict:
        """
        获取并打印 gripper 的完整状态信息

        :param verbose: 是否打印状态信息
        :return: dict 包含所有状态参数
        """
        state = {}

        # 获取基本状态
        try:
            sta = int(self._send("GET STA").split()[-1])
            obj = int(self._send("GET OBJ").split()[-1])
            pos = int(self._send("GET PRE").split()[-1])
            req_pos = int(self._send("GET POS").split()[-1])
            spe = int(self._send("GET SPE").split()[-1])
            force = int(self._send("GET FOR").split()[-1])
            flt = int(self._send("GET FLT").split()[-1])

            state = {
                "status": eStatus(sta),
                "object_status": eObjectStatus(obj),
                "position": pos,
                "requested_position": req_pos,
                "speed": spe,
                "force": force,
                "fault": eFaultCode(flt),
            }
        except Exception as e:
            if verbose:
                print(f"[Gripper] Failed to read state: {e}")
            return {}

        if verbose:
            print("=== Gripper State ===")
            print(f"Status:           {state['status'].name}")
            print(f"Object Status:    {state['object_status'].name}")
            print(f"Position:         {state['position']} / {state['requested_position']} (current/requested)")
            print(f"Speed:            {state['speed']} (0-255)")
            print(f"Force:            {state['force']} (0-255)")
            print(f"Fault:            {state['fault'].name}")
            print("=====================")

        return state
