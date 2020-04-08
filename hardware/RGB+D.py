from numpy import *
import numpy as np
import cv2 as cv
import open3d as o3d
from scipy.io import loadmat
import time


# numpy转点云格式
def np_to_pcd(pc):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    return pcd

# 从保存的校准数据文件npy中读取内参和第一张图的R和t
def read_mat(mat_path):
    mat = loadmat(mat_path)
    return mat["KK"], mat["Rc_1"], mat["Tc_1"]


def map_Dimg_to_RGBcamera(depth_image, D_cam_intrinsic, RGB_cam_intrinsic, R_D2RGB, t_D2RGB):
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
    rows, cols = depth_image.shape
    t_D2RGB = np.reshape(t_D2RGB, (3, 1))
    # ir 内参逆阵
    D_cam_intrinsic_inv = linalg.inv(D_cam_intrinsic)
    new_d_image = np.zeros((rows, cols))
    # 建立ir像面坐标，900指某一点的深度900mm，注意是 Zc [x  y 1]
    depth_oneline = depth_image.reshape((1,-1))
    u = np.arange(cols)
    u = u.repeat(rows)
    v = np.arange(rows)
    v = np.tile(v, cols)

    x = np.multiply(u, depth_oneline)
    y = np.multiply(v, depth_oneline)
    p_ir_set = np.vstack((x, y, depth_oneline))

    P_ir_set = np.dot(D_cam_intrinsic_inv, p_ir_set)
    P_ir_set_new = np.dot(R_D2RGB, P_ir_set) + t_D2RGB

    # 计算RGB视角下的深度图
    p_ir_set_new = np.dot(D_intrinsic_mtx, P_ir_set_new)
    p_ir_new_depth = p_ir_set_new[2, :]
    p_ir_new_xy = p_ir_set_new[0:2, :]
    p_ir_new_uv = (p_ir_new_xy/p_ir_new_depth)

    for uv, d in zip(p_ir_new_uv, p_ir_new_depth):
        if (0 < uv[1] < 320) and (0 < uv[0] < 240):
            new_d_image[int(uv[0])][int(uv[1])] = d

    print("对齐完成")
    new_d_image = new_d_image.astype(np.uint8)
    o3d.visualization.draw_geometries([np_to_pcd(P_ir_set.reshape((-1, 3)))])
    o3d.visualization.draw_geometries([np_to_pcd(P_ir_set_new.reshape((-1, 3)))])
    return new_d_image

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
            if pixel_depth == 0:
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
                count += 1
                x = int((p_rgb / p_rgb[2])[0])
                y = int((p_rgb / p_rgb[2])[1])

                if (0 < x < 640) and (0 < y < 480):
                    result_image[row][col] = rgb_image[y][x]
                    pointcloud.colors.append(rgb_image[y][x]/255)
                    # rgb_image[y][x] = (0, 0, 255)   # 可视化运行过程
                else:
                    result_image[row][col] = img_d[row][col]
                    pointcloud.colors.append([0.5, 0.5, 0.5])

            result_image = result_image.astype(np.uint8)
            # img_d[row][col] = 255
            # cv.imshow("img_rgb", rgb_image)
            # cv.imshow("img_d", img_d)
            # cv.imshow("img_result", result_image)
            # cv.waitKey(1)
    print("%d个点深度值大于0"%count)
    result_image = result_image.astype(np.uint8)
    tt = time.time() - start
    print("对齐完成，用时%s" % tt)
    # cl, index = pointcloud.remove_radius_outlier(3, 5)
    # pointcloud = pointcloud.select_down_sample(index)
    o3d.visualization.draw_geometries([pointcloud])
    return result_image


# 计算从D到RGB的变换
def D2RGB_Rt_from_extrinsic_matrix(D_R, D_t, RGB_R, RGB_t):
    R = np.dot(D_R.T, RGB_R)
    t = RGB_t - D_t
    return R, t


if __name__ == '__main__':
    RGB_intrinsic_mtx, RGB_R, RGB_t = read_mat("data/rgb/Calib_Results_right.mat")
    D_intrinsic_mtx, D_R, D_t = read_mat("data/D/Calib_Results_left.mat")
    R, t = D2RGB_Rt_from_extrinsic_matrix(D_R, D_t, RGB_R, RGB_t)

    rgb_image = cv.imread("data/rgb/RGB_1.bmp")
    depth_image = cv.imread('data/D/D_1.bmp', cv.IMREAD_ANYDEPTH)
    img_d = (depth_image.copy() / 20).astype(np.uint8)
    cv.imshow("img_d", img_d)
    # print(img_d.shape)
    # cv.waitKey()
    cv.imshow("img_rgb", rgb_image)
    # cv.waitKey()
    start = time.time()
    # match_image = rgb_d_to_pointcloud(depth_image, rgb_image, D_intrinsic_mtx, RGB_intrinsic_mtx, R, t)
    new_d = map_Dimg_to_RGBcamera(depth_image, D_intrinsic_mtx, RGB_intrinsic_mtx, R, t)
    print("用时%s"%(time.time()-start))
    # cv.imshow("match_result", match_image)
    cv.imshow("new_depth_img", new_d)
    cv.waitKey()



