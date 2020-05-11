from numpy import *
import numpy as np
import cv2
import open3d as o3d
from scipy.io import loadmat
import time
import glob
from kinect import kinect4


# numpy转点云格式
def np_to_pcd(pc):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    return pcd

# 从保存的校准数据文件npy中读取内参和第一张图的R和t
def read_mat(mat_path):
    mat = loadmat(mat_path)
    return mat["KK"], mat["Rc_1"], mat["Tc_1"]

class DepthImg:
    def __init__(self, filename_or_img, camera_K, mode='read_file'):
        if mode=='read_file':
            self.data = cv2.imread(filename_or_img, cv2.IMREAD_ANYDEPTH)
        elif mode=='from_img':
            self.data = filename_or_img
        self.IMG_HGT, self.IMG_WID = self.data.shape
        self.shape = self.data.shape
        self.camera_K = camera_K
        self.average_f = camera_K[0][0]/2.0+camera_K[1][1]/2.0
        self.gen_tab()  # 重新生成速算表
        return

    ## 计算速算表
    #       tab_x[u,v]=(u-u0)*K
    #       tab_y[u,v]=(v-v0)*k
    # 其中：k=1/fx=1/fy, (u0,v0)是深度图中心的像素坐标
    # 通过速算表，计算像素位置(u,v)对应的物理坐标(x,y,z)
    #       x=tab_x[u,v]*z, y=tab_y[u,v]*z
    # 注意：为了方便使用，tab_x和tab_y矩阵被拉直成向量存放
    def gen_tab(self):
        u0 = self.camera_K[0][2]
        v0 = self.camera_K[1][2]

        u = (np.arange(self.IMG_WID) - u0) / self.average_f
        v = (np.arange(self.IMG_HGT) - v0) / self.average_f

        self.tab_x = np.tile(u, self.IMG_HGT)
        self.tab_y = np.repeat(v, self.IMG_WID)
        return

    ## 从深度图img_dep计算点云，点云坐标(x,y,z)和像素位置(u,v)的对应关系为：
    #       x=img_dep[u,v]*(u-u_cent)/fx
    #       y=img_dep[u,v]*(v-v_cent)/fy
    #       z=img_dep[u,v]
    # 其中：(u-u_cent)/fx和(v-v_cent)/fy分别在速算表tab_x和tab_y中给出
    # 返回点云数据pc，每行是一个点的x/y/z坐标
    def to_numpy(self):
        pc = np.zeros((np.size(self.data), 3))
        pc[:, 0] = self.data.ravel() * self.tab_x
        pc[:, 1] = self.data.ravel() * self.tab_y
        pc[:, 2] = self.data.ravel()
        return pc

    # 转o3d点云
    def to_pcd(self):
        pc = np.zeros((self.data.size, 3))
        pc[:, 0] = self.data.ravel() * self.tab_x
        pc[:, 1] = self.data.ravel() * self.tab_y
        pc[:, 2] = self.data.ravel()
        pc = np_to_pcd(pc)
        valid_list = list(np.where(self.data.ravel() != 0)[0])
        pc_valid = o3d.geometry.PointCloud.select_down_sample(pc, valid_list)
        return pc_valid

    ## 打开文件，保存深度数据
    def open_file(self, fname='depth_frame.bin'):
        self.fp_sav = open(fname, 'wb')

    # 保存深度数据帧
    def save(self, img_dep):
        if self.fp_sav is None: self.fp_sav = open('depth_frame.bin', 'wb')
        img_dep.ravel().astype(np.float32).tofile(self.fp_sav)

    ## 关闭深度数据文件
    def close_file(self):
        self.fp_sav.close()
        self.fp_sav = None

def depth2pc(depth_img, K = np.array([572.4114, 0.0, 325.2611, 0.0, 573.57043, 242.04899, 0.0, 0.0, 1.0], dtype=float).reshape(3, 3)):
    img = DepthImg(depth_img, K, mode='from_img')
    return img.to_pcd()


# 从多组rgb+d外参计算从D到rgb的R,t
def cal_average_d2rgb_Rt(D_extrinsic_mtxs_dir, RGB_extrinsic_mtx_dir):
    root_dir = "../data/RGB+D_img/"
    calib_result_intrinsic_fname = root_dir + "intrinsic_mtx.npy"
    images = glob.glob(root_dir + '*.png')


# 获得RGB视角下的深度图
def map_Dimg_to_RGBcamera(depth_image, D_cam_intrinsic, RGB_image_shape, RGB_cam_intrinsic, R_D2RGB, t_D2RGB):
    '''
    rows, cols = depth_image.shape
    t_D2RGB = np.reshape(t_D2RGB, (3, 1))
    # ir 内参逆阵
    D_cam_intrinsic_inv = linalg.inv(D_cam_intrinsic)
    new_d_image = np.zeros((rows, cols))
    # 建立ir像面坐标，900指某一点的深度900mm，注意是 Zc [x  y 1]
    for row in range(rows):
        for col in range(cols):

            pixel_depth = depth_image[row][col]
            if pixel_depth == 0:
                continue
            p_ir = np.array([[col * pixel_depth], [row * pixel_depth], [pixel_depth]])

            P = np.dot(D_cam_intrinsic_inv, p_ir)
            P_new = np.dot(R_D2RGB, P) + t_D2RGB

            # 计算RGB视角下的深度图
            p_ir_new = np.dot(D_intrinsic_mtx, P_new)
            new_depth = p_ir_new[2]
            if new_depth > 0:
                x = int((p_ir_new / new_depth)[0])
                y = int((p_ir_new / new_depth)[1])
                if (0 < x < 320) and (0 < y < 240):
                    new_d_image[y][x] = new_depth

    '''
    D_IMG_HGT, D_IMG_WID = depth_image.shape
    RGB_IMG_HGT, RGB_IMG_WID = RGB_image_shape
    t_D2RGB = np.reshape(t_D2RGB, (3, 1))
    # ir 内参逆阵
    # D_cam_intrinsic_inv = linalg.inv(D_cam_intrinsic)
    # new_d_image = np.zeros((RGB_IMG_HGT, RGB_IMG_WID))
    new_d_image = np.zeros(RGB_image_shape)

    # 建立ir像面坐标，900指某一点的深度900mm，注意是 Zc [x  y 1]
    depth_oneline = depth_image.ravel()
    
    u0 = D_cam_intrinsic[0][2]
    v0 = D_cam_intrinsic[1][2]
    average_f = D_cam_intrinsic[0][0] / 2.0 + D_cam_intrinsic[1][1] / 2.0
    u = (np.arange(D_IMG_WID) - u0) / average_f
    v = (np.arange(D_IMG_HGT) - v0) / average_f

    tab_x = np.tile(u, D_IMG_HGT)
    tab_y = np.repeat(v, D_IMG_WID)

    P_ir_set = np.zeros((depth_image.size, 3))
    P_ir_set[:, 0] = depth_oneline * tab_x
    P_ir_set[:, 1] = depth_oneline * tab_y
    P_ir_set[:, 2] = depth_oneline
    # invalid_list1 = np.where(depth_oneline == min(depth_oneline))[0]
    # invalid_list2 = np.where(depth_oneline == max(depth_oneline))[0]
    # invalid_list = np.concatenate((invalid_list1, invalid_list2))
    # invalid_list.sort()
    
    # x = np.multiply(u, depth_oneline)
    # y = np.multiply(v, depth_oneline)
    # p_ir_set = np.vstack((x, y, depth_oneline))

    # P_ir_set = np.dot(D_cam_intrinsic_inv, p_ir_set)
    P_ir_set_new = np.dot(R_D2RGB, P_ir_set.T) + 10*t_D2RGB
    # P_ir_set_new[:, invalid_list] = 1e-16
    
    # 计算RGB视角下的深度图
    p_ir_set_new = np.dot(RGB_cam_intrinsic, P_ir_set_new)
    p_ir_new_depth = p_ir_set_new[2, :]
    p_ir_new_xy = p_ir_set_new[0:2, :]
    p_ir_new_xy = (p_ir_new_xy/p_ir_new_depth)
    
    count = 0
    for i in range(depth_image.size):
        vu = p_ir_new_xy[:, i]
        d = p_ir_new_depth[i]
        if (0 < vu[0] < RGB_IMG_WID) and (0 < vu[1] < RGB_IMG_HGT):
            count += 1
            new_d_image[int(vu[1])][int(vu[0])] = d

    print("对齐完成，新图有%s个有效像素" % count)
    origin_pc = np_to_pcd(P_ir_set.reshape((-1, 3)))
    # origin_pc = origin_pc.select_down_sample(invalid_list, invert=True)
    # origin_pc.paint_uniform_color([1, 0, 0])
    # new_pc = np_to_pcd(P_ir_set_new.T)
    # new_pc = new_pc.select_down_sample(invalid_list, invert=True)
    # new_pc.paint_uniform_color([0, 1, 0])
    # o3d.visualization.draw_geometries([origin_pc, new_pc])
    return new_d_image

# 在图片上画格子
def draw_lines(img):
    height, width = img.shape[0:2]
    length = 150
    for j in range(width):
        if j % length == 0:
            img[:, j] = 150
            # cv2.imshow("drawing", img)
            # cv2.waitKey(100)
    for i in range(height):
        if i % length == 0:
            img[i, :] = 150
            # cv2.imshow("drawing", img)
            # cv2.waitKey(100)
    # cv2.destroyWindow("drawing")
    return img

# 深度图染色
def rgb_d_to_pointcloud(depth_image, rgb_image, D_cam_intrinsic, RGB_cam_intrinsic, R_D2RGB, t_D2RGB):
    rows, cols = depth_image.shape
    t_D2RGB = np.reshape(t_D2RGB, (3, 1))
    pointcloud = o3d.geometry.PointCloud()
    # ir 内参逆阵
    D_cam_intrinsic_inv = linalg.inv(D_cam_intrinsic)

    img_d = depth_image.copy().astype(np.uint8)
    result_image = np.zeros((rows, cols, 3))

    start = time.time()
    count = 0
    # 建立ir像面坐标，900指某一点的深度900mm，注意是 Zc [x  y 1]
    for row in range(rows):
        for col in range(cols):

            pixel_depth = depth_image[row][col]
            if pixel_depth == 65300 or pixel_depth == 0:
                continue
            p_ir = np.array([[col * pixel_depth], [row * pixel_depth], [pixel_depth]])

            # print ("---- Pir ---")
            P_ir = np.dot(D_cam_intrinsic_inv, p_ir)
            pointcloud.points.append(P_ir)
            # print (P_ir)
            P_rgb = np.dot(R_D2RGB, P_ir) + t_D2RGB
            # print ("---- P rgb ---")
            # print (P_rgb)

            # 从RGB图中找颜色
            p_rgb = np.dot(RGB_cam_intrinsic, P_rgb)
            if p_rgb[2] > 0:
                x = int((p_rgb / p_rgb[2])[0])
                y = int((p_rgb / p_rgb[2])[1])

                if (0 < x < cols) and (0 < y < rows):
                    count += 1
                    result_image[row][col] = rgb_image[y][x]
                    
                    pointcloud.colors.append(rgb_image[y][x][::-1]/255)
                    # rgb_image[y][x] = (0, 0, 255)   # 可视化运行过程
                else:
                    result_image[row][col] = img_d[row][col]
                    pointcloud.colors.append([1, 0, 0])

            result_image = result_image.astype(np.uint8)
            # img_d[row][col] = 255
            # cv2.imshow("img_rgb", rgb_image)
            # cv2.imshow("img_d", img_d)
            # cv2.imshow("img_result", result_image)
            # cv2.waitKey(1)
    print("%d个点在彩色图中找到"%count)
    result_image = result_image.astype(np.uint8)
    tt = time.time() - start
    print("对齐完成，用时%s" % tt)
    # cl, index = pointcloud.remove_radius_outlier(3, 5)
    # pointcloud = pointcloud.select_down_sample(index)
    o3d.visualization.draw_geometries([pointcloud])
    return result_image


# 深度图转点云
def depth_2_pc(depth_image, cam_mtx):
    IMG_HGT, IMG_WID = depth_image.shape
    
    # 建立ir像面坐标，900指某一点的深度900mm，注意是 Zc [x  y 1]
    depth_oneline = depth_image.ravel()
    
    u0 = cam_mtx[0][2]
    v0 = cam_mtx[1][2]
    average_f = cam_mtx[0][0] / 2.0 + cam_mtx[1][1] / 2.0
    u = (np.arange(IMG_WID) - u0) / average_f
    v = (np.arange(IMG_HGT) - v0) / average_f
    
    tab_x = np.tile(u, IMG_HGT)
    tab_y = np.repeat(v, IMG_WID)
    
    P_ir_set = np.zeros((depth_image.size, 3))
    P_ir_set[:, 0] = depth_oneline * tab_x
    P_ir_set[:, 1] = depth_oneline * tab_y
    P_ir_set[:, 2] = depth_oneline
    return np_to_pcd(P_ir_set)


# 深度图转点云，带颜色
def depth_2_colored_pc(depth_image, color_image, cam_mtx):
    IMG_HGT, IMG_WID = depth_image.shape
    
    # 建立ir像面坐标，900指某一点的深度900mm，注意是 Zc [x  y 1]
    depth_oneline = depth_image.ravel()
    R = color_image[:, :, 0].reshape(-1, 1)
    G = color_image[:, :, 1].reshape(-1, 1)
    B = color_image[:, :, 2].reshape(-1, 1)
    colors = np.hstack((B, G, R))
    
    
    u0 = cam_mtx[0][2]
    v0 = cam_mtx[1][2]
    average_f = cam_mtx[0][0] / 2.0 + cam_mtx[1][1] / 2.0
    u = (np.arange(IMG_WID) - u0) / average_f
    v = (np.arange(IMG_HGT) - v0) / average_f
    
    tab_x = np.tile(u, IMG_HGT)
    tab_y = np.repeat(v, IMG_WID)
    
    P_ir_set = np.zeros((depth_image.size, 3))
    P_ir_set[:, 0] = depth_oneline * tab_x
    P_ir_set[:, 1] = depth_oneline * tab_y
    P_ir_set[:, 2] = depth_oneline
    
    pc = np_to_pcd(P_ir_set)
    colors = o3d.utility.Vector3dVector(colors /255)
    pc.colors = colors
    return pc

# 计算从D到RGB的变换
def D2RGB_Rt_from_extrinsic_matrix(rgb_transformation, d_transformation):
    rgb_R = rgb_transformation[0:3, 0:3]
    rgb_t = rgb_transformation[0:3, 3]
    d_R = d_transformation[0:3, 0:3]
    d_t = d_transformation[0:3, 3]
    R = np.dot(rgb_R, d_R.T)
    t = rgb_t - d_t
    return R, t

def kinect4_visualizer():
    cam = kinect4()
    img_color, img_depth = cam.get_capture()
    cameramtx = cam.cameramtx
    pc_colored = depth_2_colored_pc(img_depth, img_color, cameramtx)
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pc_colored)
    while 1:
        print("*")
        img_color, img_depth = cam.get_capture()
        pc_colored.points = depth_2_colored_pc(img_depth, img_color, cameramtx).points
        pc_colored.colors = depth_2_colored_pc(img_depth, img_color, cameramtx).colors
        vis.update_geometry(pc_colored)
        vis.poll_events()
        vis.update_renderer()

def main():
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
    
    # 读取D到rgb空间变换
    R = np.load("../data/kinect2/D2RGB_rotation_mtx.npy")
    t = np.load("../data/kinect2/D2RGB_translation_vec.npy")
    print("D到rgb的空间变换R，t：\n", R, '\n', t)
    
    # rgb相机去畸
    rgb_image = cv2.imread('../data/kinect2/rgb/%s.png' % picture_num)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(rgb_intrinsic_mtx, rgb_distort, rgb_image.shape[1::-1], 1)
    map1, map2 = cv2.initUndistortRectifyMap(rgb_intrinsic_mtx, rgb_distort, None, newcameramtx, rgb_image.shape[1::-1],
                                             cv2.CV_32FC1)
    rgb_image_undistort = cv2.remap(rgb_image, map1, map2, cv2.INTER_NEAREST)
    rgb_intrinsic_mtx = newcameramtx
    print("new rgb_camera mtx:\n", newcameramtx)
    
    depth_image = cv2.imread('../data/kinect2/depth/%s.png' % picture_num, cv2.IMREAD_ANYDEPTH)
    img_d = cv2.imread('../data/kinect2/gray/%s.png' % picture_num)
    
    # 深度相机去畸
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(d_intrinsic_mtx, d_distort, img_d.shape[1::-1], 1)
    map1, map2 = cv2.initUndistortRectifyMap(d_intrinsic_mtx, d_distort, None, newcameramtx, img_d.shape[1::-1],
                                             cv2.CV_32FC1)
    img_d_undistort = cv2.remap(img_d, map1, map2, cv2.INTER_NEAREST)
    depth_image_undistort = cv2.remap(depth_image, map1, map2, cv2.INTER_NEAREST)
    d_intrinsic_mtx = newcameramtx
    print("new d_camera mtx:\n", newcameramtx)
    
    # cv2.imshow("rgb", rgb_image)
    
    rgb_show = draw_lines(rgb_image_undistort)
    cv2.imshow("rgb_undistort", rgb_show)
    # cv2.imshow("d", img_d)
    cv2.imshow("d_undistort", img_d_undistort)
    
    start = time.time()
    # match_image = rgb_d_to_pointcloud(depth_image, rgb_image, d_intrinsic_mtx, rgb_intrinsic_mtx, R, t)
    new_d = map_Dimg_to_RGBcamera(depth_image_undistort, d_intrinsic_mtx, rgb_image.shape[0:2], rgb_intrinsic_mtx, R, t)
    print("用时%s" % (time.time() - start))
    
    # img = DepthImg(new_d, rgb_intrinsic_mtx, mode='from_img')
    # pc = img.to_pcd()
    # pc.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries([pc])
    
    d_uint8 = depth_image.copy().astype(np.uint8)
    new_d_uint8 = new_d.copy().astype(np.uint8)
    cv2.imshow("origin_d", d_uint8)
    new_d_show = draw_lines(new_d_uint8)
    cv2.imshow("new_d", new_d_show)
    
    cv2.waitKey(0)

if __name__ == '__main__':
    main()
    



