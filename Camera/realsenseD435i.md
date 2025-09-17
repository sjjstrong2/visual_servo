# RealSense D435i相机控制模块技术文档

## 概述

本模块提供了一个Python类[Camera](vscode-file://vscode-app/d:/Program%20Files/Microsoft%20VS%20Code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)，用于控制Intel RealSense D435i深度相机。该模块封装了相机的初始化、数据获取和图像显示功能，支持获取彩色图像、深度图像以及相机内参。模块适用于机器人视觉应用，如物体检测和深度感知。

### 主要功能

* 初始化相机并配置流参数（分辨率、帧率）
* 获取对齐的彩色和深度图像数据
* 计算并返回相机内参矩阵
* 显示彩色和深度图像（使用OpenCV）
* 支持深度图像的颜色映射显示

### 依赖项

* `numpy`: 用于数组操作和矩阵计算
* `pyrealsense2`: Intel RealSense SDK，用于相机控制
* `cv2` (OpenCV): 用于图像处理和显示

## 类结构

### Camera 类

#### 构造函数

初始化相机对象，设置图像尺寸和帧率。

* 参数:
  * `width`: 图像宽度 (默认640)
  * `height`: 图像高度 (默认480)
  * `fps`: 帧率 (默认15)
* 属性:
  * `im_height`: 图像高度
  * `im_width`: 图像宽度
  * `fps`: 帧率
  * `intrinsics`: 相机内参矩阵 (3x3)
  * `scale`: 深度尺度
  * `pipeline`: RealSense管道对象

#### 公共方法

##### 连接和配置

* `connect() -> None`
  * 配置深度和彩色流，启动相机流
  * 获取相机内参和深度尺度
  * 输出: 打印深度尺度和连接状态

##### 数据获取

* `get_data() -> Tuple[np.ndarray, np.ndarray]`
  * 获取一对对齐的彩色和深度帧
  * 返回: (color_image, depth_image)
    * `color_image`: BGR格式的彩色图像 (numpy数组)
    * `depth_image`: 深度图像 (float32, 扩展为3维)

##### 图像显示

* `plot_image() -> None`
  * 获取图像数据并显示彩色和深度图像
  * 使用JET颜色映射显示深度图像
  * 窗口显示5秒后自动关闭

##### 内参获取

* `get_intrinsics(rgb_profile) -> np.ndarray`
  * 从RealSense配置文件计算相机内参矩阵
  * 参数: `rgb_profile` (RealSense流配置文件)
  * 返回: 3x3内参矩阵 (numpy数组)
    * 格式: [[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]]

## 使用示例

```python
from Camera.realsenseD435i import Camera

# 创建相机实例 (默认640x480, 15fps)
camera = Camera()

# 获取图像数据
color_img, depth_img = camera.get_data()
print(f"Color image shape: {color_img.shape}")
print(f"Depth image shape: {depth_img.shape}")

# 显示图像
camera.plot_image()

# 获取相机内参
intrinsics = camera.intrinsics
print(f"Camera intrinsics:\n{intrinsics}")
```

## 注意事项

* 相机必须正确连接并安装RealSense SDK
* 深度图像以毫米为单位，可通过 `scale`属性转换为米
* 图像对齐使用彩色流作为参考，确保深度和彩色图像像素对应
* 显示功能使用OpenCV，窗口会阻塞5秒
* 内参矩阵基于RealSense提供的原始参数计算
* 适用于D415/D435系列相机，但代码中注释提到D415
* 错误处理: 如果相机未连接或配置失败，会抛出异常
* 性能: 高分辨率或高帧率可能影响实时性能

## 版本信息

* 日期: 2025年9月1日
* 兼容性: Python 3.x, RealSense SDK 2.x
* 硬件: Intel RealSense D435i/D415
* 库版本: numpy, pyrealsense2, opencv-python
