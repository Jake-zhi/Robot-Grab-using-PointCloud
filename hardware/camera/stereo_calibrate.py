import numpy as np
import cv2
import RGB_D


# 计算从D到RGB的变换
def D2RGB_Rt_from_extrinsic_matrix(rgb_transformation, d_transformation):
    rgb_R = rgb_transformation[0:3, 0:3]
    rgb_t = rgb_transformation[0:3, 3]
    d_R = d_transformation[0:3, 0:3]
    d_t = d_transformation[0:3, 3]
    R = np.dot(rgb_R, d_R.T)
    t = rgb_t - d_t
    return R, t

# 文件记录点击位置
# 点击灰度图，记录三维坐标x, y, z
def OnMouseAction_on_gray(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("左键点击灰度图%s,%s" % (x, y))
        click_col, click_row = x, y
        f = open("depth_camera_choosen_points.txt", 'a')
        z = depth_image_undistort[click_row][click_col]
        x = click_col * z
        y = click_row * z
        f.write("%s %s %s\n" % (x, y, z))
        
# 文件记录点击位置
# 点击彩色图，记录像素x, y
def OnMouseAction_on_RGB(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("左键点击彩色图%s,%s" % (x, y))
        f = open("RGB_camera_choosen_pixels.txt", 'a')
        f.write("%s %s\n" % (x, y))
        

# 手动选取对应点，计算两相机空间变换
def cal_Rt_developing():
    # 清空写入的文件
    f = open("depth_camera_choosen_points.txt", 'w')
    f.close()
    f = open("RGB_camera_choosen_pixels.txt", 'w')
    f.close()
    
    # 读取标定结果
    picture_num = 27
    rgb_intrinsic_distort = np.load('../data/kinect2/rgb/intrinsic_mtx.npy', allow_pickle=True).item()
    rgb_intrinsic_mtx = rgb_intrinsic_distort['intrinsic_mtx']
    rgb_distort = rgb_intrinsic_distort['distort']
    d_intrinsic_distort = np.load('../data/kinect2/depth/intrinsic_mtx.npy', allow_pickle=True).item()
    d_intrinsic_mtx = d_intrinsic_distort['intrinsic_mtx']
    d_distort = d_intrinsic_distort['distort']
    print("rgb_mtx:\n", rgb_intrinsic_mtx)
    print("rgb_distort:\n", rgb_distort)
    print("d_mtx:\n", d_intrinsic_mtx)
    print("d_distort:\n", d_distort)
    
    # rgb相机去畸
    rgb_image = cv2.imread('../data/kinect2/rgb/%s.png' % picture_num)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(rgb_intrinsic_mtx, rgb_distort, rgb_image.shape[1::-1], 1)
    map1, map2 = cv2.initUndistortRectifyMap(rgb_intrinsic_mtx, rgb_distort, None, newcameramtx, rgb_image.shape[1::-1], cv2.CV_32FC1)
    rgb_image_undistort = cv2.remap(rgb_image, map1, map2, cv2.INTER_NEAREST)
    rgb_intrinsic_mtx = newcameramtx
    print("new rgb_camera mtx:\n", newcameramtx)
    
    depth_image = cv2.imread('../data/kinect2/depth/%s.png' % picture_num, cv2.IMREAD_ANYDEPTH)
    img_d = cv2.imread('../data/kinect2/gray/%s.png' % picture_num)
    
    # 深度相机去畸
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(d_intrinsic_mtx, d_distort, img_d.shape[1::-1], 1)
    map1, map2 = cv2.initUndistortRectifyMap(d_intrinsic_mtx, d_distort, None, newcameramtx, img_d.shape[1::-1], cv2.CV_32FC1)
    img_d_undistort = cv2.remap(img_d, map1, map2, cv2.INTER_NEAREST)
    global depth_image_undistort
    depth_image_undistort = cv2.remap(depth_image, map1, map2, cv2.INTER_NEAREST)
    d_intrinsic_mtx = newcameramtx
    print("new d_camera mtx:\n", newcameramtx)
    
    cv2.imshow("rgb_undistort", rgb_image_undistort)
    cv2.imshow("d_undistort", img_d_undistort)
    
    d_uint8 = depth_image_undistort.copy().astype(np.uint8)
    cv2.imshow("origin_d", d_uint8)
    cv2.setMouseCallback("d_undistort", OnMouseAction_on_gray)
    cv2.setMouseCallback("rgb_undistort", OnMouseAction_on_RGB)
    cv2.waitKey(0)
    

# 从两相机外参计算两相机空间变换，手动微调
def cal_T_from_extrinsic_mtx():
    root_dir = "../data/kinect2/"
    rgb_rvecs = np.load(root_dir + 'rgb/rvecs.npy')
    d_rvecs = np.load(root_dir + 'gray/rvecs.npy')
    rgb_tvecs = np.load(root_dir + 'rgb/tvecs.npy')
    d_tvecs = np.load(root_dir + 'gray/tvecs.npy')
    
    rvecs = rgb_rvecs - d_rvecs
    rvec = np.mean(rvecs, axis=0)
    # rvec = rvecs[0]
    
    rotation_mtx = cv2.Rodrigues(rvec)[0]
    print("rvec:\n", rvec)
    print("rotation_mtx:\n", rotation_mtx)
    print("***********************************")
    
    tvecs = rgb_tvecs - d_tvecs
    # print(tvecs)
    tvec = np.mean(tvecs, axis=0)
    # tvec = tvecs[0]
    print("tvec:\n", tvec)
    
    np.save(root_dir+"D2RGB_rotation_mtx.npy", rotation_mtx)
    np.save(root_dir+"D2RGB_translation_vec.npy", tvec)
    temp = tvec
    while 1:
        RGB_D.main()
        b = input("手动调整：")
        b1, b2, b3 = b.split()
        temp = temp + np.array([[float(b1)], [float(b2)], [float(b3)]])
        np.save(root_dir + "D2RGB_translation_vec.npy", temp)
        RGB_D.main()
        save = input("输入r反悔。")
        if save == 'r':
            np.save(root_dir + "D2RGB_translation_vec.npy", tvec)

if __name__ == '__main__':
    developing()