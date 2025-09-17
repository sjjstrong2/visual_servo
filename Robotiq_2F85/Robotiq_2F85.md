# Robotiq夹爪控制模块技术文档

## 概述

本模块提供了一个Python类 `RobotiqGripper`，用于通过socket通信直接控制Robotiq的夹爪设备。该模块基于字符串命令与夹爪进行交互，支持激活、移动、校准等操作。模块已通过HAND-E型号测试。

### 主要功能

* 连接和断开与夹爪的socket通信
* 激活和重置夹爪
* 控制夹爪的位置、速度和力度
* 自动校准位置范围
* 实时查询夹爪状态和位置
* 线程安全的命令发送（使用锁机制）

### 依赖项

* `socket`: 用于网络通信
* `threading`: 用于线程锁，确保命令原子性
* `time`: 用于延时操作
* `enum`: 用于定义枚举类型
* `typing`: 用于类型注解（Union, Tuple, OrderedDict）

## 类结构

### RobotiqGripper 类

#### 常量

* **WRITE VARIABLES (可读写)** :
* `ACT`: 激活标志 (0=重置, 1=激活)
* `GTO`: 执行移动 (1=执行)
* `ATR`: 自动释放 (0=禁用, 1=启用)
* `ADR`: 自动释放方向 (0=关闭, 1=打开)
* `FOR`: 力度 (0-255)
* `SPE`: 速度 (0-255)
* `POS`: 位置 (0-255, 0=打开)
* **READ VARIABLES (只读)** :
* `STA`: 状态 (0=重置, 1=激活中, 3=激活)
* `PRE`: 位置请求 (最后命令的位置)
* `OBJ`: 对象检测 (0=移动中, 1=外夹, 2=内夹, 3=无对象)
* `FLT`: 故障 (0=正常, 其他=错误)
* **ENCODING** : 编码方式 ('UTF-8')

#### 枚举

* **GripperStatus** : 夹爪状态枚举
* `RESET = 0`
* `ACTIVATING = 1`
* `ACTIVE = 3`
* **ObjectStatus** : 对象状态枚举
* `MOVING = 0`
* `STOPPED_OUTER_OBJECT = 1`
* `STOPPED_INNER_OBJECT = 2`
* `AT_DEST = 3`

#### 构造函数

* []()
* []()
* []()
* []()

初始化夹爪对象，设置默认参数：

* socket: None
* command_lock: 线程锁
* 位置/速度/力度范围: 0-255

#### 公共方法

##### 连接管理

* `connect(hostname: str, port: int, socket_timeout: float = 2.0) -> None`
  * 连接到指定主机和端口的夹爪
  * 参数: hostname (IP或主机名), port (端口号), socket_timeout (超时时间，默认2.0秒)
* `disconnect() -> None`
  * 断开与夹爪的连接

##### 激活和重置

* `activate(auto_calibrate: bool = True) -> None`
  * 激活夹爪，清除故障标志
  * 参数: auto_calibrate (是否自动校准，默认True，但代码中注释掉了)
* `is_active() -> bool`
  * 检查夹爪是否激活
  * 返回: True=激活, False=未激活

##### 位置查询

* `get_min_position() -> int`
  * 获取最小位置 (打开位置)
  * 返回: 最小位置值
* `get_max_position() -> int`
  * 获取最大位置 (关闭位置)
  * 返回: 最大位置值
* `get_open_position() -> int`
  * 获取打开位置 (同get_min_position)
* `get_closed_position() -> int`
  * 获取关闭位置 (同get_max_position)
* `is_open() -> bool`
  * 检查是否完全打开
  * 返回: True=打开, False=未打开
* `is_closed() -> bool`
  * 检查是否完全关闭
  * 返回: True=关闭, False=未关闭
* `get_current_position() -> int`
  * 获取当前位置
  * 返回: 当前位置值 (0-255)

##### 移动控制

* `move(position: int, speed: int, force: int) -> Tuple[bool, int]`
  * 发送移动命令到指定位置
  * 参数: position (目标位置), speed (速度), force (力度)
  * 返回: (命令发送成功, 实际位置)
* `move_and_wait_for_pos(position: int, speed: int, force: int) -> Tuple[int, ObjectStatus]`
  * 移动到位置并等待完成
  * 参数: 同move
  * 返回: (最终位置, 对象状态)

##### 校准

* `auto_calibrate(log: bool = True) -> None`
  * 自动校准打开/关闭位置
  * 参数: log (是否打印结果，默认True)

#### 私有方法

* `_set_vars(var_dict: OrderedDict[str, Union[int, float]]) -> bool`
  * 设置多个变量
* `_set_var(variable: str, value: Union[int, float]) -> bool`
  * 设置单个变量
* `_get_var(variable: str) -> int`
  * 获取变量值
* `_is_ack(data: bytes) -> bool`
  * 检查ACK响应
* `_reset() -> None`
  * 重置夹爪


## 使用示例

```python
from Robotiq_2F85 import RobotiqGripper

# 创建夹爪实例
gripper = RobotiqGripper()

# 连接到夹爪 (假设IP为192.168.1.100, 端口63352)
gripper.connect("192.168.1.100", 63352)

# 激活夹爪
gripper.activate()

# 移动到打开位置
gripper.move_and_wait_for_pos(gripper.get_open_position(), 100, 50)

# 移动到关闭位置
gripper.move_and_wait_for_pos(gripper.get_closed_position(), 100, 50)

# 获取当前位置
current_pos = gripper.get_current_position()
print(f"Current position: {current_pos}")

# 断开连接
gripper.disconnect()
```


## 注意事项

* 所有操作需要先连接到夹爪
* 激活后才能进行移动操作
* 位置、速度、力度值会被自动裁剪到有效范围 (0-255)
* 移动操作是异步的，使用 `move_and_wait_for_pos`等待完成
* 线程安全：内部使用锁确保命令原子性
* 错误处理：网络超时或无效响应会抛出异常
* 测试环境：仅在HAND-E型号上测试过
* 编码：使用UTF-8，但ASCII也兼容
