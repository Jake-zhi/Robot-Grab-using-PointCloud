#!/usr/bin/python3
# coding=utf-8
from visualize_interaction.pc_visulization import open3d_visualize
from process_data.point_cloud_preprocess import *
import time


def execute_ransac_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size,
                max_iteration=1000000, max_validation=5000000, fitness_threshold=0.9, inlier_rmse_threshold=10):
    ransac_n = 4
    distance_threshold = voxel_size*3

    print("Apply ransac global run_registration with distance threshold %.3f" \
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
    print(":: Apply fast global run_registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result


def refine_registration(source, target, registration_result, voxel_size):
    distance_threshold = voxel_size * 3
    print("Point-to-plane ICP run_registration to refine the alignment.")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, registration_result.transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    return result


def run_registration(model, scene, direction_file_dir="data/cam_dirs", voxel_size=8):
    # 读取数据
    # 恐龙
    # voxel_size = 5
    # src_pc_filename = "data/model_ds.ply"
    # dst_pc_filename = "data/scene_ds.ply"

    # 茶壶模型匹配场景
    # src_pc_filename = "data/model_pot.ply"
    # dst_pc_filename = "data/scene_duck.ply"

    # 鸭子模型匹配场景
    # voxel_size = 5
    # src_pc_filename = "data/model_duck.ply"
    # dst_pc_filename = "data/scene_duck.ply"

    # 鸭子场景（用mask）匹配模型
    # src_pc_filename = "data/scene_only_duck.ply"
    # dst_pc_filename = "data/model_duck_full.ply"
    # 读取点云、降采样

    src_pc, src_pc_down, src_pc_down_fpfh = model[0], model[1], model[2]
    dst_pc, dst_pc_down, dst_pc_down_fpfh = scene[0], scene[1], scene[2]
    # src_pc_from_n_directions, src_pc_down_fpfh_n_directions = get_2_5D_pc_8Direction(src_pc_down, src_pc_down_fpfh)
    # src_pc_from_n_directions, src_pc_down_fpfh_n_directions = get_2_5D_pc_DirectionFromFile(src_pc_down, src_pc_down_fpfh, direction_file_dir, mode='runtime')
    src_pc_from_n_directions, src_pc_down_fpfh_n_directions = [src_pc_down], [src_pc_down_fpfh]
    # 配准
    # ransac
    final_result = o3d.registration.RegistrationResult()
    final_model = o3d.geometry.PointCloud()
    for i in range(len(src_pc_from_n_directions)):
        this_25D_model = src_pc_from_n_directions[i]
        this_25D_fpfh = src_pc_down_fpfh_n_directions[i]
        start = time.time()
        this_registration_result = execute_ransac_global_registration(this_25D_model, dst_pc_down, this_25D_fpfh, dst_pc_down_fpfh, voxel_size)
        # fast
        # this_registration_result = execute_fast_global_registration(this_25D_model, dst_pc_down, this_25D_fpfh, dst_pc_down_fpfh, voxel_size)
        print("run_registration fitness is %.3f\ninlier_rmse is %.3f" % (this_registration_result.fitness, this_registration_result.inlier_rmse))

        # ICP
        this_refined_result = refine_registration(this_25D_model, dst_pc_down, this_registration_result, voxel_size)
        # refined_result = registration_result
        print("processing time: ", time.time() - start)
        print("refined fitness is %.3f\ninlier_rmse is %.3f" % (this_refined_result.fitness, this_refined_result.inlier_rmse))


        # run_registration icp 二选一
        if(this_refined_result.inlier_rmse <= this_registration_result.inlier_rmse and this_refined_result.fitness>=this_registration_result.fitness-0.2):
            this_result = this_refined_result
            print("use refined rusult")
        elif this_refined_result.fitness == 0 or this_refined_result.inlier_rmse == 0 or this_refined_result.fitness<this_registration_result.fitness-0.5:
            print("wrong match")
            continue
        else:
            this_result = this_registration_result
            print("use run_registration rusult")
        # 多个方向2.5D模型取最佳
        if (this_result.inlier_rmse < final_result.inlier_rmse and this_result.fitness > final_result.fitness):
            final_result = this_result
            final_model = this_25D_model
            print("找到更好结果。")
        else:
            print("不是更好结果。")

    print('''********************************************************************************
final fitness is %.3f\ninlier_rmse is %.3f''' % (final_result.fitness, final_result.inlier_rmse))

    # 计算、显示结果
    R = final_result.transformation
    # src_trans_np=np.dot(src_np,R[:3,:3].T)+R[:3,3].T

    final_model_trans = final_model.transform(R)
    full_model_trans = src_pc.transform(R)

    dst_pc_down.paint_uniform_color([0, 1, 0])
    final_model_trans.paint_uniform_color([1, 0, 0])
    open3d_visualize([final_model_trans, dst_pc_down])

    dst_pc.paint_uniform_color([0, 1, 0])
    full_model_trans.paint_uniform_color([1, 0, 0])
    open3d_visualize([full_model_trans, dst_pc])
    return final_result

if __name__ == '__main__':
    run_registration(model ="data/model/model_pot.ply", scene ="data/scene/scene_duck&pot.ply")
