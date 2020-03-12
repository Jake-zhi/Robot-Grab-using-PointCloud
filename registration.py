#!/usr/bin/python3
# coding=utf-8
import open3d as o3d
import numpy as np
from pc_visulization import plt_visualize
from point_cloud_preprocess import *


def execute_ransac_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size,
                max_iteration=500000, max_validation=250000, fitness_threshold=0.8, inlier_rmse_threshold=10):
    ransac_n = 4
    distance_threshold = voxel_size*3

    print("Apply ransac global registration with distance threshold %.3f" \
          % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), ransac_n, [
            o3d.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            o3d.registration.RANSACConvergenceCriteria(max_iteration, max_validation, fitness_threshold, inlier_rmse_threshold))

    return result


def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 2
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result


def refine_registration(source, target, registration_result, voxel_size):
    distance_threshold = voxel_size * 3
    print("Point-to-plane ICP registration to refine the alignment.")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, registration_result.transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    return result


if __name__ == "__main__":
    # 读取数据
    # 加载数据并作格式转化
    # source = np_to_pcd(np.genfromtxt('data/ds.csv', delimiter=','))
    # target = np_to_pcd(np.genfromtxt('data/dst.csv', delimiter=','))

    # 恐龙成功匹配
    # voxel_size = 5
    # src_pc = o3d.io.read_point_cloud("data/model_ds.ply")
    # dst_pc = o3d.io.read_point_cloud("data/scene_ds.ply")

    # 茶壶模型匹配场景
    # src_pc = o3d.io.read_point_cloud("data/model_pot.ply")
    # dst_pc = o3d.io.read_point_cloud("data/scene_duck.ply")

    # 鸭子模型匹配场景
    # voxel_size = 5
    # src_pc = o3d.io.read_point_cloud("data/model_duck.ply")
    # dst_pc = o3d.io.read_point_cloud("data/scene_duck.ply")

    # 鸭子场景（用mask）匹配模型
    # src_pc = o3d.io.read_point_cloud("data/scene_only_duck.ply")
    # dst_pc = o3d.io.read_point_cloud("data/model_duck_full.ply")

    # np.fromfile("C:/Users/Zhang Lantao/Desktop/000000.bin").reshape(-1,3)
    # dst_pc = np_to_pcd(dst_np)
    voxel_size = 4
    src_pc, dst_pc, src_pc_down, dst_pc_down, src_pc_down_fpfh, dst_pc_down_fpfh = prepare_data("data/model_pot.ply", "data/scene_duck.ply", voxel_size=voxel_size)

    src_pc_8direction, src_pc_fpfh_8direction = get_2_5D_pc_8Direction(src_pc_down, src_pc_down_fpfh)

    # 配准
    # ransac
    registration_result = execute_ransac_global_registration(src_pc_down, dst_pc_down, src_pc_down_fpfh, dst_pc_down_fpfh, voxel_size)
    # fast
    # registration_result = execute_fast_global_registration(src_pc_down, dst_pc_down, src_pc_fpfh, dst_pc_fpfh, voxel_size)
    print("registration fitness is %.3f\ninlier_rmse is %.3f" % (registration_result.fitness, registration_result.inlier_rmse))

    # ICP
    refined_result = refine_registration(src_pc_down, dst_pc_down, registration_result, voxel_size)
    # refined_result = registration_result
    print("refined fitness is %.3f\ninlier_rmse is %.3f" % (refined_result.fitness, refined_result.inlier_rmse))

    if(refined_result.inlier_rmse <= registration_result.inlier_rmse and refined_result.fitness>=registration_result.fitness):
        final_result = refined_result
        print("use refined rusult")
    else:
        final_result = registration_result
        print("use registration rusult")

    # 计算、显示结果
    R = refined_result.transformation
    # src_trans_np=np.dot(src_np,R[:3,:3].T)+R[:3,3].T

    src_trans_pc_down = src_pc_down.transform(R)
    src_trans_pc = src_pc.transform(R)

    # np.savetxt('out.csv', src_trans_np, fmt='%.8f', delimiter=',', newline='\n')
    # o3d.io.write_point_cloud("C:/Users/Zhang Lantao/Desktop/dst.ply", dst_pc, write_ascii=True)

    dst_pc_down.paint_uniform_color([0, 1, 0])
    src_trans_pc_down.paint_uniform_color([1, 0, 0])
    plt_visualize([src_trans_pc_down, dst_pc_down])

    dst_pc.paint_uniform_color([0, 1, 0])
    src_trans_pc.paint_uniform_color([1, 0, 0])
    plt_visualize([src_trans_pc, dst_pc])
