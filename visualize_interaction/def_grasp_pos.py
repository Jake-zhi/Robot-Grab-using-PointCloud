#!/usr/bin/python3
# coding=utf-8
'''

'''

editor = "notepad.exe"  # 系统默认文本编辑器(根据实际情况选择)
model_num = 8
model_dir = "../data/model/using/%s/" % model_num
filename = model_dir + "%s.ply" % model_num  # 模型文件
grasp_pose_filename = model_dir + "grab_pose_transformation.npy"
voxel_size = 5  # 模型体素降采样大小(与主程序一致)

import open3d as o3d
import numpy as np
from process_data.point_cloud_preprocess import prepare_data, np_to_pcd


def pick_points(pcd):
    print("*******************************************************************")
    print("[shift + 左键单击]选择三点，顺序为[抓取点1， 抓取点2， 平面参考点]")
    print("[shift + 右键单击]后退")
    print("选择结束以后按Esc退出")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


def main():
    model, _, _ = prepare_data(filename, voxel_size=-1)
    pick_list = pick_points(model)
    points = np.asarray(model.points)[pick_list]
    grasp_point1 = points[0, :]
    grasp_point2 = points[1, :]
    aux_point    = points[2, :]
    '''
    从三点计算夹爪坐标系法向
    gp1->gp2方向为x轴正方向
    aux->gp1 叉乘 aux->gp2 （平面法向）为y轴正方向
    即z轴沿着夹爪指向。
    '''
    # 由三点计算夹爪坐标轴方向
    aux_gp1 = grasp_point1 - aux_point
    aux_gp2 = grasp_point2 - aux_point
    y_positive = np.cross(aux_gp1, aux_gp2)
    y_positive /= np.linalg.norm(y_positive)
    x_positive = grasp_point2 - grasp_point1
    x_positive /= np.linalg.norm(x_positive)
    z_positive = np.cross(x_positive, y_positive)
    # 计算夹爪坐标原点
    grasp_origin = np.average(points[0:2, :], axis=0)
    grasp_pose = np.vstack((grasp_origin,
                            grasp_origin + x_positive,
                            grasp_origin + y_positive,
                            grasp_origin + z_positive))

    # 计算夹爪相对于模型的位姿
    new_frame = np_to_pcd(np.vstack((grasp_origin + x_positive,
                                     grasp_origin + y_positive,
                                     grasp_origin + z_positive)))

    origin_frame = np_to_pcd(np.array([[1, 0, 0],
                                       [0, 1, 0],
                                       [0, 0, 1]]))
    corrs = o3d.utility.Vector2iVector(np.array([[0, 0], [1, 1], [2, 2]]))
    estimator = o3d.registration.TransformationEstimationPointToPoint()
    transformation = estimator.compute_transformation(origin_frame, new_frame, corrs)

    np.save(grasp_pose_filename, transformation)
    
    #画出夹爪坐标系
    d = np.linalg.norm(np.asarray(model.get_max_bound()) - np.asarray(model.get_min_bound()))
    grasp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=d / 5)
    grasp_frame.transform(transformation)

    o3d.visualization.draw_geometries([model, grasp_frame])


if __name__ == "__main__":
    main()
