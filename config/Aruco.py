import cv2
import cv2.aruco as aruco

# 选择字典
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# 设置要生成的ID和尺寸（单位像素）
marker_id = 0
marker_size = 200  # 200x200 像素

# 生成 marker
img = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# 保存为 PNG 图片
cv2.imwrite("aruco_marker_id0.png", img)

# 可视化
cv2.imshow("marker", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
