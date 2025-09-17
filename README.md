# visual_servo

好嘞 👍 现在从仿真过渡到 **UR 机械臂 + 相机 + 视觉伺服 (PBVS)** 的真机部署，要特别注意安全、标定、接口、限速。下面我给你梳理一个完整的操作步骤和注意事项，分阶段走。

---

# 🚀 真机部署步骤（逐步）

## 1. 环境准备

1. **硬件**
   * UR5e / UR10 等机械臂
   * 工具端安装相机（或固定相机，但需手眼标定）
   * 标记物（如 ArUco/AprilTag 板）
   * 工控机/PC，运行 Ubuntu + ROS/RTDE 接口
2. **软件**
   * Python ≥ 3.8
   * OpenCV（含 `cv2.aruco`, `solvePnP`）
   * ur_rtde (官方 C++/Python API)
   * Numpy, SciPy
   * （可选）ROS + MoveIt (若要统一运动规划)

---

## 2. 相机标定

1. **相机内参 & 畸变系数**
   * 用棋盘格或 ChArUco 标定，得到 `K, distCoeffs`。
   * 标定结果保存成 `.yaml` 文件。
   * **注意** ：真机部署时，失真校正很关键，否则 pose 解会漂。
2. **手眼标定 (extrinsics)**
   * 目标：得到 $T_{\text{tcp}_\text{cam}}$ （相机相对于 TCP 工具坐标的外参）。
   * 常用方法：
     * **标志板法** ：在世界坐标固定 marker，多姿态采集，solve hand-eye (`cv2.calibrateHandEye`)。
     * **直接测量法** ：粗略测量相机在 TCP 的位置、姿态（但精度低）。
   * 结果：保存为 4×4 齐次矩阵，供控制转换使用。

---

## 3. 机械臂接口配置

1. **UR 控制模式**
   * 确认机械臂固件支持  **RTDE 接口** （UR ≥ v5.9）。
   * 开启 `Remote Control` 模式。
   * 建议以 **速度控制（速度 Twist）** 模式运行，而不是位置点。
2. **速度限制**
   * 在代码中设置线速度上限 0.05 m/s、角速度 0.3 rad/s（避免突发高速）。
   * 在 UR teach pendant 上设置  **速度/力矩限制** 。
3. **安全设置**
   * 设置安全围栏或虚拟防护区。
   * 确保急停按钮可随时按下。

---

## 4. 控制回路（PBVS）

1. **图像处理**
   * 相机采集图像。
   * 检测 marker → 得到 `image_points`。
   * 调用 `cv2.solvePnP(object_points, image_points, K, dist)` 得到 $(R,t)$。
2. **坐标变换**
   * 得到 $T_{\text{cam}_\text{marker}}$。
   * 转换到 TCP：

     Ttcp_marker=Ttcp_cam⋅Tcam_markerT_{\text{tcp}\_\text{marker}} = T_{\text{tcp}\_\text{cam}} \cdot T_{\text{cam}\_\text{marker}}
   * 转换到 Base：

     Tbase_marker=Tbase_tcp⋅Ttcp_markerT_{\text{base}\_\text{marker}} = T_{\text{base}\_\text{tcp}} \cdot T_{\text{tcp}\_\text{marker}}
3. **误差计算**
   * 与期望位姿 $T_{\text{base}_\text{marker}}^{des}$ 做差 → $(e_t, e_r)$。
   * 用 PBVS 控制器计算 $(v_{lin}, v_{ang})$。
4. **发送控制**
   * 获取当前 Jacobian $J$ via `rtde_receive.getInverseKinematicsJacobian()`。
   * 用 `PBVSController.map_cartesian_to_joint_velocities(v_lin, v_ang, J)` 计算 $q_dot$。
   * 发送关节速度命令 `rtde_control.servoJ(q_dot, ...)` 或 `speedJ(q_dot, ...)`。

---

## 5. 测试流程

1. **仿真验证**
   * 先在 Gazebo/PyBullet/MoveIt 模拟跑通，确保 marker pose 解和控制环路正确。
2. **真机 dry-run**
   * 关闭相机反馈，给一个固定小速度，验证 UR 机械臂能安全跟随。
   * 打开视觉，但只打印 pose，不控制，确认姿态解正常。
3. **闭环控制**
   * 启动视觉伺服，但控制增益设得很小（Kp ~ 0.1）。
   * 确认机器人能朝 marker 逐渐移动而不会发疯。
4. **逐步提升**
   * 增大增益，测试完整闭环。
   * 加入滤波（指数平滑、EKF）避免抖动。

---

# ⚠️ 部署注意事项（重点）

* **安全第一** ：
* 增益不要过大，避免瞬间高速冲击。
* 永远有人监控急停按钮。
* 尝试先在虚拟保护模式运行（UR 有 simulation mode）。
* **单位一致性** ：
* 确保 `object_points` 单位（mm 或 m）与控制环路统一。
* 通常：marker 边长 mm → PnP 输出 `tvec` 单位 mm → 转换成 m 后传给控制器。
* **奇异与退化** ：
* 如果 marker 出现在图像边缘或部分丢失，PnP 可能发散。
* 建议用 `solvePnPRansac` 并检查 `retval`。
* 若 pose 估计失败，保持上一步的速度 or 停止。
* **噪声滤波** ：
* 对 pose 或 $(v_{lin}, v_{ang})$ 使用低通滤波（α 滤波器）。
* 避免机械臂来回抖动。
* **实时性** ：
* UR 控制回路推荐 50–125 Hz。
* 相机帧率要跟上（30+ FPS），否则有延迟 → 引入 D-term 可改善。

---

# ✅ 最终流程简表

1. 标定相机 (K, dist)。
2. 标定手眼 (T_tcp_cam)。
3. 启动 UR，开启 RTDE。
4. 相机采集图像，检测 marker。
5. `solvePnP` 解 $T_{cam_marker}$。
6. 坐标转换 → $T_{base_marker}$。
7. PBVS 控制器算速度命令。
8. 雅可比映射 → 关节速度。
9. 发送 `speedJ(q_dot)` 控制 UR。
10. 实时循环（50–100 Hz），安全监控。

---

要不要我帮你把这个流程直接写成  **部署 checklist** （操作清单，照着一步步做就行的）？这样你上线调试时不会漏掉关键环节。






好 — 我把你整个工程里 **必须注意／非常可能需要替换** 的代码位置都列清楚了：每个文件、具体函数/方法、为什么要改、应该改成什么（示例片段或替代思路），以及部署前后的核验清单和测试用例。你可以照着这个清单逐项替换／校准，逐步把工程从“通用模板”变成“对你真机可工作的版本”。

我把内容按优先级排序（ **最关键先做** ），每项给出：文件、函数、需要改动的具体地方、示例改法、测试/验证方法。开始：

---

# 一、最关键（真实机器人接口与坐标系） —— 必须修改，否则真机会危险或不工作

## 1) 文件：`ur_visual_servo/ur_interface.py`

### 需要替换 / 注意的函数

* `__init__`：如何初始化 RTDE / 你用的 UR python 包（`rtde_control`/`rtde_receive`、`ur_rtde`、`urx` 等）。
* `get_current_base_tcp(self) -> Optional[np.ndarray]`： **必须实现为你 RTDE 的真实读取** 。返回  **4×4 numpy.ndarray** ，并明确单位（通常 RTDE 返回是  **米** ）。
* `get_jacobian(self) -> Optional[np.ndarray]`： **必须实现** ，返回形状 `(6, n_joints)` 的雅可比（单位要和 controller 的速度一致，通常线速度 m/s、角速度 rad/s）。
* `send_joint_velocity(self, q_dot: np.ndarray)`： **必须实现** ，将 `q_dot`（rad/s）发送到机器人（或转换为你的 API 需要的格式）。
* 可选：`send_cartesian_velocity(self, linear, angular, dt)`（如果你有直接发送笛卡尔 twist 给 robot 的接口）
* 可选：`get_num_joints()`（方便安全停用时发送零速度）

### 为什么要改

* 这是跟真机交互的唯一通路。不同 UR/python SDK 方法名、单位和签名差异很大，必须与真实环境一致（例如 `rtde_control.speedL()` vs `rtde_control.servoCartesianL()` vs `rtde_control.servoJ()` 等）。

### 示例：常见 `rtde_control` 的实现参考（**示例** — 请用你真实库替换）

```python
# 伪代码示例：把下面替换进 ur_interface.py，按你实际库调整函数名/参数
import rtde_control
import rtde_receive

class URAdapter:
    def __init__(self, cfg):
        self.simulate = cfg.ur_host is None
        if not self.simulate:
            self.rtde_ctl = rtde_control.RTDEControlInterface(cfg.ur_host)
            self.rtde_rcv = rtde_receive.RTDEReceiveInterface(cfg.ur_host)

    def get_current_base_tcp(self):
        pose = self.rtde_rcv.getActualTCPPose()  # typically [x,y,z,rx,ry,rz] in meters
        if pose is None:
            return None
        t = np.array(pose[:3])
        rvec = np.array(pose[3:])
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3,:3] = R
        T[:3,3] = t
        return T  # meters

    def get_jacobian(self):
        jac = self.rtde_rcv.getActualJacobian()  # check returned shape
        if jac is None:
            return None
        arr = np.asarray(jac, dtype=float)
        # adapt reshape/transpose depending on your API
        if arr.size == 36:
            arr = arr.reshape((6,6))
        return arr  # shape (6,n)

    def send_joint_velocity(self, q_dot):
        # for rtde_control, might use speedJ or servoj depending on API
        self.rtde_ctl.speedJ(q_dot.tolist(), a=0.5, t=0.02)  # example: speedJ expects list, accel, time
```

> **务必** ：把这些伪代码中的函数名、单位（m vs mm）、参数（是否要 duration、加速度）改为你装的 SDK 文档要求。

### 验证 / 测试

* 在机器人处于安全模式（低速或示教模式）下调用 `get_current_base_tcp()` 并打印结果，确认：平移单位是**米**还是 **毫米** ，旋转是 rvec 还是 quaternion，或返回 4×4 矩阵。
* 调用 `get_jacobian()` 并打印 shape/数值（小幅位移测试，观察雅可比是否合理）。
* 写一个小脚本单独调用 `send_joint_velocity`（发送很小速度）并确认机器人运动且没有报警。先用 very small q_dot（例如 0.01 rad/s）。

---

## 2) 文件：`ur_visual_servo/main.py`（主循环） — 必改点

### 需要替换 / 注意的地方

* RTDE 读取的 `T_base_tcp` 单位处理（代码现在假定 robot 返回 **m** 并转换为 mm 用于与相机链做乘法）。如果你的 UR 返回 mm，需要移除乘 1000 操作，或改成一致单位。
* `ur.get_jacobian()` 方法调用与后续使用：你要保证 `J @ q_dot = [v_lin; v_ang]` 的单位匹配（v_lin 为 m/s）。
* 最终发送命令：现在的流程把笛卡尔速度通过雅可比映射为 `q_dot` 并调用 `ur.send_joint_velocity(q_dot)`；你必须保证 `send_joint_velocity` 的签名匹配（很多 API 还需要加速度/时间参数或用 `servoJ` 接口）。

### 典型替换点（示例片段）

```py
# 在主循环里
T_base_tcp_robot = ur.get_current_base_tcp()
# 检查返回单位
# ... 如果返回的是米，则与 cam->marker(mm) 链条保持一致需要转单位
# 或者改为把 camera chain 全部转成 meters（更简单）
```

 **建议改法（更好的一致性）** ：把相机 -> marker 的结果 (`solvePnP`) 直接以 **米** 为单位输出（把 `marker_size_mm` 改成 `marker_size_m` 或在读取后立即除以 1000）。这样整个控制代码内部统一使用  **米** ，减少换算错误。

### 为什么推荐把视觉数据改成米

* 机器人 RTDE 很常见地使用 **米** 单位，若把视觉链全部转为 meters，避免在不同环节多处转换导致出错。

### 验证 / 测试

* 在主循环中临时打印每个阶段的单位：`T_cam_marker` 单位、`T_base_tcp` 单位、`current_T_base` 单位。确认一致。
* 在仿真/慢速真机下运行一次循环，观察日志，确保没有 1000 倍的 scale 错误（常见 symptom：机器人径向飞出去或根本不动）。

---

# 二、相机与位姿估计 —— 需校准/替换的代码点

## 3) 文件：`ur_visual_servo/camera.py`

### 需要注意 / 可替换

* `_load_camera_params(path)`：确保 `camera_matrix.npz` 文件格式与你的文件一致（键名 `camera_matrix` 和 `dist`）。
* `build_marker_object_points(marker_size_mm)`（在 utils）与 `solvePnP` 的单位一致。
* `estimate_pose(...)` 使用的 `solvePnP` flag：对于 ArUco 方形 marker 推荐 `SOLVEPNP_IPPE_SQUARE`，若你发现二义性或不稳定，可试 `ITERATIVE` 并 `solvePnPRansac` + refine。

### 推荐改法（建议）

* **把 object points 单位改为米** （修改 `marker_size_mm` → `marker_size_m`，或在 `camera.estimate_pose` 里把 tvec 除以 1000），统一整个系统用  **米** 。
* 在检测前做 `cornerSubPix` 精化（提升精度）：

```py
# 在 detect 后：
for c in corners:
    cv2.cornerSubPix(gray, c, (5,5), (-1,-1), criteria)
```

### 验证 / 测试

* 将相机对准 marker 的已知真实位置（用尺量），比较 `solvePnP` 输出的平移是否与真实值接近（误差在几毫米级或可接受范围）。
* 在多角度下采样 20 帧，统计 tvec 的均值和标准差（查看抖动量）。

---

# 三、手眼标定与外参 —— 必核对

## 4) 文件：`ur_visual_servo/utils.py` & `config.py`

### 需要替换 / 注意

* `cfg.hand_eye_path`：把真实 `hand_eye.npy` 上传到该路径并在 `config.yaml` 指定 `hand_eye_convention`（`T_tcp_cam` 或 `T_cam_tcp`）。 **必须确认外参方向** 。
* `cam_marker_to_base_marker` 的链 `T_base_tcp @ T_tcp_cam @ T_cam_marker` 只在 `T_tcp_cam` 表示 “从 TCP 到 camera” 时成立。若你外参是相反的，代码会求逆（我之前已有处理），但你必须核对。

### 验证

* 在机器人静止且 TCP 在已知位姿时做一次观测，计算理论 `T_base_marker`（用手眼 / tcp pose）并和视觉计算的 `T_base_marker` 比较位置和旋转误差。差不应太大（几个 cm 以内或更小，视标定精度）。如果差异很大，手眼标定有问题。

---

# 四、控制器参数与数值稳定 —— 必调参

## 5) 文件：`ur_visual_servo/controller.py`

### 需要检查/替换

* `linear_gain`, `angular_gain`, `max_linear_speed`, `max_angular_speed`：这些参数要根据真实机器人的惯量、速度能力、任务要求调。
* `kd`, `alpha`（如果在构造器传入）是否启用：真实系统中推荐 `kd>0` 和小 `alpha`（例如 `kd=0.02`、`alpha=0.05`）以减少抖动。

### 验证 / 调参流程

1. 把 `max_*` 设得非常小（0.01 m/s，0.02 rad/s），观察行为。
2. 确认方向正确（给一个已知偏差，观察速度方向）。
3. 逐步放大 `Kp`，确保不过振；若出现振荡降低 `Kp` 或加 `Kd`。

---

# 五、保存/加载路径与法规

## 6) 文件：`config.py` 与 `bin/*`

### 替换/校验

* `camera_matrix_path`：确保路径存在并格式正确（`.npz`）。
* `desired_pose_path`：部署时确认是否覆盖重要数据（做好备份）。
* `base_tcp_path`：在没有 robot 连接时可用作替代，但生产时一般由 robot 实时提供 `T_base_tcp`。

---

# 六、GUI / Key 操作 —— 小心在线运行时误触发

## 7) 文件：`main.py`（GUI 部分）

### 必改

* 在真机运行时**建议关闭 GUI 的保存键 `q`** 或把保存文件写到备份目录，并限制按键操作权限，避免无人值守误按。
* 加入明确的“运行模式”(dry-run / engage)，避免在错误模式下发送真实命令。

---

# 七、日志/监控与急停 —— 强烈建议添加/替换

## 8) 新增或调整代码

* 在 `ur_interface.py` 中添加异常捕获：若发送命令失败或 robot 报错，立即调用急停或快速停止接口（如果 API 支持）。
* 把关键日志（当前位姿、速度命令、雅可比 condition number）写到文件或 ROS topic 以便回放分析。

示例：

```py
try:
    ur.send_joint_velocity(q_dot)
except Exception as e:
    logging.exception("Send q_dot failed — trying estop")
    ur.estop()  # 如果你的 adapter 有这个方法
```

---

# 八、测试清单（部署前必须通过）

把下面流程当成强制 checklist：

1. **接口测试（机器人）**
   * 能成功连接 RTDE。
   * `get_current_base_tcp()` 返回值合理（打印并确认单位）。
   * `get_jacobian()` 返回 shape 合理。
   * `send_joint_velocity()` 在低速下安全执行。
2. **视觉测试（相机与 PnP）**
   * 在不同距离/角度下检测 marker 并打印 `tvec`（统计均值/方差）。
   * 检查亚像素与畸变处理是否生效。
3. **坐标链一致性测试**
   * 把机器人移动到几个已知 Pose（记录 TCP pose），对比视觉估算出的 `T_base_marker` 与理论，误差在可接受范围。
4. **闭环慢速测试**
   * 在低增益与低限速下运行视觉伺服，观察机器人平稳收敛到目标。
5. **极端情形测试**
   * 掩盖 marker（simulate lost）并观察安全停行为。
   * 标记在图像边缘/近距离/远距离下的反应与鲁棒性。
6. **安全**
   * 将急停按钮接入测试并验证能立即停止代码的动作。
   * 设置安全围栏和工作区限制（软件和硬件）。

---

# 九、替换优先级总结（按顺序操作）

1. **实现/替换 `ur_interface.py` 的 RTDE 接口方法（最重要）** ：`get_current_base_tcp()`、`get_jacobian()`、`send_joint_velocity()`、`get_num_joints()`。
2. **在主循环里统一单位（建议把视觉链转成米）并确认 `main.py` 的单位匹配** 。
3. **保证 `camera.py` 输出单位与主循环期望一致** （建议直接输出 meters）。
4. **调整 controller 的 gain/limit 并启用小的 kd/alpha** 。
5. **校验手眼外参 `hand_eye.npy` 与 `hand_eye_convention`** 。
6. **实现/加强错误处理、日志与急停** 。
7. **做全面的接口/闭环/极端测试** （见上面测试清单）。

---

# 十、常见易错点（提醒你避免）

* 忽略单位（mm vs m）导致 1000 倍错误；这是最常见、最危险的错误。
* 错把 `T_tcp_cam` 和 `T_cam_tcp` 弄反（方向反了输出会完全错误）。
* 直接把 controller 输出当作 joint speeds 发送（忘了用雅可比或误用雅可比的转置/伪逆）。
* 忽略 robot API 的 `duration`/`accel` 参数（发送瞬时速度不带时间会被忽略或行为异常）。
* 在真实机器人上没做速度上界与加速度限制就直接调高增益。

---

# 附：我可以直接为你做的改动（你任选）

* A — 把 `ur_interface.py` 用你真实的 RTDE 库实现（你把库名与你看到的函数/返回格式发来，我直接给你精确实现）。
* B — 把相机链统一成**米**并把所有相关代码（camera、utils、main、controller）改好并打包成可运行版本。
* C — 写一个自动化“上线前检查脚本”（检查 RTDE 可用、camera 可用、hand-eye 存在、units 一致性、打印 summary 报告）。

你选 A / B / C（或多个），或者把你当前 `ur_interface.py` 发上来，我直接基于你的现有代码改写并给回完整文件。
