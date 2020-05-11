#coding:utf-8
import cv2
import numpy as np
import glob

def draw_chease_board(height = 1200, width = 1800, length = 200):
    image = np.zeros((height, width), dtype=np.uint8)
    print(image.shape[0], image.shape[1])

    for j in range(width):
        for i in range(height):
            if (int(i / length) + int(j / length)) % 2:
                image[i, j] = 255
        # 白边
        # image[:, 0:94] = 255
        # image[:, width - 100:width] = 255
        # image[0:94, :] = 255
        # image[height - 94:height, :] = 255

    cv2.imwrite("chess_board.png", image)
    cv2.imshow("chess", image)
    cv2.waitKey(0)
# draw_chease_board()


root_dir = "../data/kinect2/rgb/"
calib_result_intrinsic_fname = root_dir + "intrinsic_mtx.npy"
tvecs_fname = root_dir + "tvecs.npy"
rvecs_fname = root_dir + "rvecs.npy"
images = glob.glob(root_dir + '*.png')
print("list file complete.")
# 找棋盘格角点
# 阈值
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#棋盘格模板规格
w = 8
h = 5
len = 2.87 #黑白格长度
# 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)，去掉Z坐标，记为二维矩阵
objp = np.zeros((w*h,3), np.float32)
objp[:, :2] = np.mgrid[0:w*len:len,0:h*len:len].T.reshape(-1, 2)
# 储存棋盘格角点的世界坐标和图像坐标对
objpoints = [] # 在世界坐标系中的三维点
imgpoints = [] # 在图像平面的二维点

i = 0
j = 0
for fname in images:
    # i += 1
    # if i%50:
    #     continue
    # j += 1
    print(fname)
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray', gray)
    cv2.waitKey(1)
    # 找到棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, (w,h),None)
    # 如果找到足够点对，将其存储起来
    if ret == True:
        cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners)
        # 将角点在图像上显示
        cv2.drawChessboardCorners(img, (w, h), corners, ret)
        cv2.imshow('findCorners', img)
        cv2.waitKey(1)
cv2.destroyAllWindows()
# print("有多少张图：", j)

# 标定
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("校准完成，内参：")
print(mtx)
save_dict = {'intrinsic_mtx': mtx, 'distort': dist}
np.save(calib_result_intrinsic_fname, save_dict)
np.save(rvecs_fname, rvecs)
np.save(tvecs_fname, tvecs)

extrinsic_mtxs = []
R_mtxs = []
for fname, rvec, tvec in zip(images, rvecs, tvecs):
    rotation_mtx = cv2.Rodrigues(rvec)[0]
    extrinsic_mtx = np.hstack((rotation_mtx, tvec))
    extrinsic_mtx = np.vstack((extrinsic_mtx, np.array([0, 0, 0, 1])))
    R_mtxs.append(rotation_mtx)
    extrinsic_mtxs.append(extrinsic_mtx)
    print("外参：")
    print(extrinsic_mtx)
    calib_result_extrinsic_fname = fname.replace('.png', 'extrinsic_mtx.npy')
    save_dict = {'tvec': tvec, 'rvec': rvec, 'extrinsic_mtx': extrinsic_mtx, 'rotation_mtx': rotation_mtx}
    np.save(calib_result_extrinsic_fname, save_dict)


# print("done")
# 去畸变
img2 = cv2.imread(root_dir + '115.png')
h,  w = img2.shape[:2]
dst = cv2.undistort(img2, mtx, dist)
cv2.imshow("distort", dst)
# cv2.imwrite('calibresult.png',dst)

# rgb相机去畸
rgb_image = img2
cv2.imshow("rgb", rgb_image)
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, rgb_image.shape[1::-1], 1)
map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, rgb_image.shape[1::-1], cv2.CV_32FC1)
rgb_image_undistort = cv2.remap(rgb_image, map1, map2, cv2.INTER_NEAREST)
rgb_intrinsic_mtx = newcameramtx
print("new rgb_camera mtx:\n", newcameramtx)
cv2.imshow("rgb_undistort", rgb_image_undistort)
cv2.waitKey(0)



