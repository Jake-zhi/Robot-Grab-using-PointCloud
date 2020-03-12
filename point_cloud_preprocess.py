#!/usr/bin/python3
# coding=utf-8
import numpy as np
import open3d as o3d

import pc_visulization
from about_os import *


# numpy转点云格式
def np_to_pcd(pc):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    return pcd


# 点云格式转numpy
def pcd_to_np(pcd): return np.asarray(pcd.points)


# 读取、预处理数据
# 传入至少一个文件名
# 传入一个文件名，返回src_pc, src_pc_down, src_pc_fpfh
# 传入两个文件你，返回src_pc, dst_pc, src_pc_down, dst_pc_down, src_pc_fpfh, dst_pc_fpfh
def prepare_data(src_filename, dst_filename="", voxel_size=5):
    print("Open file %s,:" % src_filename)
    src_pc = o3d.io.read_point_cloud(src_filename)
    src_pc_down, src_pc_fpfh = preprocess_point_cloud(src_pc, voxel_size)
    if not dst_filename == "":
        print("Open file %s,:" % dst_filename)
        dst_pc = o3d.io.read_point_cloud(dst_filename)
        dst_pc_down, dst_pc_fpfh = preprocess_point_cloud(dst_pc, voxel_size)
        return src_pc, dst_pc, src_pc_down, dst_pc_down, src_pc_fpfh, dst_pc_fpfh

    else:
        print("No dst_pc, only read one point cloud:%s" % src_filename)
        return src_pc, src_pc_down, src_pc_fpfh


# 预处理
# 降采样,剔除离群点,提取FPFH特征
def preprocess_point_cloud(pcd, voxel_size, remove_outlier=False):
    # 降采样
    pcd_down = pcd.voxel_down_sample(voxel_size)
    # pcd_down = pcd
    # 离群点剔除
    if (remove_outlier):
        cl, ind = pcd_down.remove_radius_outlier(nb_points=25, radius=3 * voxel_size)
        # display_inlier_outlier(pcd_down, ind)
        inlier_cloud = pcd_down.select_down_sample(ind)
    else:
        inlier_cloud = pcd_down

    # 估计法向量方向
    if len(inlier_cloud.normals) == 0:
        radius_normal = voxel_size * 2
        inlier_cloud.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    # 计算FPFH特征
    radius_feature = voxel_size * 5
    inlier_cloud_fpfh = o3d.registration.compute_fpfh_feature(
        inlier_cloud,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return inlier_cloud, inlier_cloud_fpfh


# 离群点展示
def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_down_sample(ind)
    outlier_cloud = cloud.select_down_sample(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    pc_visulization.plt_visualize([inlier_cloud, outlier_cloud])


# 对模型的八个方向取2.5D点云
def get_2_5D_pc_8Direction(src_pc, src_fpfh):
    result_pc = []
    result_fpfh = []
    d = np.linalg.norm(np.asarray(src_pc.get_max_bound()) - np.asarray(src_pc.get_min_bound()))
    cam_loc = [[d / 2, d / 2, d / 2],
               [d / 2, d / 2, -d / 2],
               [d / 2, -d / 2, d / 2],
               [d / 2, -d / 2, -d / 2],
               [-d / 2, d / 2, d / 2],
               [-d / 2, d / 2, -d / 2],
               [-d / 2, -d / 2, d / 2],
               [-d / 2, -d / 2, -d / 2]]
    for cam_loc_i in cam_loc:
        src_removed, point_list = src_pc.hidden_point_removal(cam_loc_i, 100 * d)

        src_pc_removed = o3d.geometry.PointCloud()
        src_pc_removed.points = src_removed.vertices
        result_pc.append(src_pc_removed)

        src_fpfh_removed = o3d.registration.Feature()
        src_fpfh_removed.data = src_fpfh.data[:, point_list]
        result_fpfh.append(src_fpfh_removed)

        # 显示
        src_pc.paint_uniform_color([1, 0, 0])
        src_pc_removed.paint_uniform_color([0, 1, 0])
        camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=d / 5, origin=cam_loc_i)
        pc_visulization.open3d_visualize([src_pc_removed, src_pc, camera_frame])
    return result_pc, result_fpfh


# 对模型指定方向取2.5D点云
def get_2_5D_pc_DirectionFromFile(src_pc, src_fpfh, direction_file_dir):
    file_list = for_files_in_dir(direction_file_dir)
    result_pc = []
    result_fpfh = []
    d = np.linalg.norm(np.asarray(src_pc.get_max_bound()) - np.asarray(src_pc.get_min_bound()))
    for file in file_list:
        cam_loc_i = np.loadtxt(file) * d / 2
        src_removed, point_list = src_pc.hidden_point_removal(cam_loc_i, 100 * d)

        src_pc_removed = o3d.geometry.PointCloud()
        src_pc_removed.points = src_removed.vertices
        result_pc.append(src_pc_removed)

        src_fpfh_removed = o3d.registration.Feature()
        src_fpfh_removed.data = src_fpfh.data[:, point_list]
        result_fpfh.append(src_fpfh_removed)

        # 显示
        src_pc.paint_uniform_color([1, 0, 0])
        src_pc_removed.paint_uniform_color([0, 1, 0])
        camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=d / 5, origin=cam_loc_i)
        pc_visulization.open3d_visualize([src_pc_removed, src_pc, camera_frame])
    return result_pc, result_fpfh


def prepare_model_and_open3d_visualize(filename, voxel_size=4):
    src_pc, _, _ = prepare_data(filename, voxel_size=voxel_size)
    pc_visulization.open3d_visualize([src_pc])


def prepare_model_and_plt_visualize(filename, voxel_size=4):
    src_pc, _, _ = prepare_data(filename, voxel_size=voxel_size)
    pc_visulization.plt_visualize([src_pc])


if __name__ == "__main__":
    model_pc, model_pc_down, model_down_fpfh = prepare_data("data/model_duck.ply")
    get_2_5D_pc_DirectionFromFile(model_pc_down, model_down_fpfh, "data/cam_dirs")
