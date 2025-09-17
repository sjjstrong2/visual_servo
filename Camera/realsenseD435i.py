# 导入必要的库
import numpy as np  # 用于数组操作
import pyrealsense2 as rs  # RealSense SDK
import cv2  # OpenCV用于图像处理

class Camera(object):
    """
    RealSense D435i相机控制类
    用于初始化相机、获取图像数据和显示图像
    """

    def __init__(self, width=640, height=480, fps=30):
        """
        构造函数：初始化相机参数并连接相机
        :param width: 图像宽度 (默认640)
        :param height: 图像高度 (默认480)
        :param fps: 帧率 (默认30)
        """
        self.im_height = height  # 图像高度
        self.im_width = width    # 图像宽度
        self.fps = fps           # 帧率
        self.rgb_intrinsics = None  # RGB相机内参矩阵
        self.scale = None        # 深度尺度
        self.pipeline = None     # RealSense管道对象
        self.connect()           # 连接相机
        # color_img, depth_img = self.get_data()  # 可选：获取初始数据
        # print(color_img, depth_img)  # 可选：打印数据信息

    def connect(self):
        """
        连接并配置RealSense相机
        设置深度和彩色流，获取相机信息和内参
        """
        # 配置深度和彩色流
        self.pipeline = rs.pipeline()  # 创建管道
        config = rs.config()  # 创建配置对象
        config.enable_stream(rs.stream.depth, self.im_width, self.im_height, rs.format.z16, self.fps)  # 启用深度流
        config.enable_stream(rs.stream.color, self.im_width, self.im_height, rs.format.bgr8, self.fps)  # 启用彩色流

        # 启动流
        cfg = self.pipeline.start(config)  # 开始流式传输

        # 获取设备信息
        device = cfg.get_device()  # 获取设备对象
        name = device.get_info(rs.camera_info.name)  # 获取设备名称

        # 获取内参
        rgb_profile = cfg.get_stream(rs.stream.color)    # RGB流配置文件
        depth_profile = cfg.get_stream(rs.stream.depth)  # 深度流配置文件
        self.rgb_intrinsics = self.get_intrinsics(rgb_profile)    # RGB内参
        self.depth_intrinsics = self.get_intrinsics(depth_profile)  # 深度内参

        # 获取深度尺度
        self.scale = cfg.get_device().first_depth_sensor().get_depth_scale()  # 深度尺度因子
        print("camera name:", name)  # 打印相机名称
        print("camera rgb_intrinsics:", self.rgb_intrinsics)  # 打印RGB内参
        print("camera depth_intrinsics:", self.depth_intrinsics)  # 打印深度内参
        print("camera depth scale:", self.scale)  # 打印深度尺度
        print("D435i have connected ...")  # 连接成功提示

    def get_data(self):
        """
        获取对齐的彩色和深度图像数据
        :return: (color_image, depth_image) 彩色图像和深度图像的numpy数组
        """
        # 等待帧数据
        frames = self.pipeline.wait_for_frames()  # 阻塞等待同步帧

        # 对齐深度到彩色
        align = rs.align(align_to=rs.stream.color)  # 创建对齐对象
        aligned_frames = align.process(frames)  # 执行对齐
        aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐深度帧
        color_frame = aligned_frames.get_color_frame()  # 获取彩色帧
        # 未对齐选项（注释）
        # depth_frame = frames.get_depth_frame()  # 原始深度帧
        # color_frame = frames.get_color_frame()  # 原始彩色帧

        # 转换为numpy数组
        depth_image = np.asanyarray(aligned_depth_frame.get_data(), dtype=np.float32)  # 深度图像转为float32
        # depth_image *= self.scale  # 可选：乘以尺度因子
        depth_image = np.expand_dims(depth_image, axis=2)  # 扩展维度为(H, W, 1)
        color_image = np.asanyarray(color_frame.get_data())  # 彩色图像转为数组
        return color_image, depth_image  # 返回图像数据

    def plot_image(self):
        """
        获取图像数据并显示彩色和深度图像
        对深度图像应用颜色映射，拼接显示5秒
        """
        color_image, depth_image = self.get_data()  # 获取图像数据
        # 对深度图像应用颜色映射（需先转为8位）
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)  # 应用JET颜色映射

        depth_colormap_dim = depth_colormap.shape  # 深度彩色映射尺寸
        color_colormap_dim = color_image.shape     # 彩色图像尺寸

        # 如果分辨率不同，调整彩色图像大小
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                             interpolation=cv2.INTER_AREA)  # 调整大小
            images = np.hstack((resized_color_image, depth_colormap))  # 水平拼接
        else:
            images = np.hstack((color_image, depth_colormap))  # 直接拼接
        # 显示图像
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)  # 创建窗口
        cv2.imshow('RealSense', images)  # 显示拼接图像
        # cv2.imwrite('color_image.png', color_image)  # 可选：保存彩色图像
        cv2.waitKey(5000)  # 等待5秒

    def get_intrinsics(self, rgb_profile):
        """
        从流配置文件获取相机内参矩阵
        :param rgb_profile: 流配置文件
        :return: 3x3内参矩阵
        """
        raw_intrinsics = rgb_profile.as_video_stream_profile().get_intrinsics()  # 获取原始内参
        print("camera intrinsics:", raw_intrinsics)  # 打印原始内参
        # 内参矩阵格式：
        # [[fx, 0, ppx],
        #  [0, fy, ppy],
        #  [0, 0, 1]]
        # intrinsics = np.array([615.284,0,309.623,0,614.557,247.967,0,0,1]).reshape(3,3)  # 示例：640x480
        intrinsics = np.array([raw_intrinsics.fx, 0, raw_intrinsics.ppx,  # 构建内参矩阵
                               0, raw_intrinsics.fy, raw_intrinsics.ppy,
                               0, 0, 1]).reshape(3, 3)
        return intrinsics  # 返回内参矩阵

# 主程序入口
if __name__ == '__main__':
    mycamera = Camera()  # 创建相机实例
    # mycamera.get_data()  # 可选：获取数据
    mycamera.plot_image()  # 显示图像
    intrinsics = mycamera.rgb_intrinsics  # 获取内参
    print(intrinsics)  # 打印内参
    # print(mycamera.intrinsics)  # 可选：打印