import numpy as np
import open3d as o3d

# numpy转点云格式
def np_to_pcd(pc):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    return pcd

# 相机坐标系下的点坐标，从kinect4可视化程序中选点读取
p_cam_1 = np.array([-0.17, -0.21, 0.5])
p_cam_2 = np.array([0.16, -0.2, 0.51])
p_cam_3 = np.array([-0.16,  -0.029, 0.43])
p_cam_4 = np.array([0.073, -0.033, 0.44])
points_cam_np = np.vstack((p_cam_1, p_cam_2, p_cam_3, p_cam_4))
points_cam = np_to_pcd(points_cam_np)

corrs = o3d.utility.Vector2iVector(np.array([[0, 0], [1, 1], [2, 2], [3, 3]]))

# 上面四个点对应的机械臂坐标系中的位置，z为固定的传送带高度
p_robot_1 = np.array([-0.6704695899475599, 0.12283119197297386, 0.7])
p_robot_2 = np.array([-0.6607762944888836, -0.2115394973925324, 0.7])
p_robot_3 = np.array([-0.8455789810156867, 0.09889199990517654, 0.7])
p_robot_4 = np.array([-0.8463394979528687, -0.1271267205717529, 0.7])
points_robot_np = np.vstack((p_robot_1, p_robot_2, p_robot_3, p_robot_4))
points_robot = np_to_pcd(points_robot_np)

estimator = o3d.registration.TransformationEstimationPointToPoint()
T = estimator.compute_transformation(points_cam, points_robot, corrs)
print(T)
np.save("T_cam_to_base.npy", T)
