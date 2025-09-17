# 读取图像
frame = cv2.imread("test.jpg")
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# 选择相同字典
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# 检测
detector = aruco.ArucoDetector(aruco_dict, parameters)
corners, ids, rejected = detector.detectMarkers(gray)

# 可视化
if ids is not None:
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    cv2.imshow("Detected Markers", frame_markers)
    cv2.waitKey(0)
