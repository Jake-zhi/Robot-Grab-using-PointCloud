import sys
from RGB_D import depth_2_pc, depth_2_colored_pc
from pos_estimation.registration import run_registration
from process_data import depth_img_copy_mask
from retinanet.detection import detect
import cv2
import numpy as np
from process_data.about_os import for_files_in_folder
import open3d as o3d
from process_data.point_cloud_preprocess import prepare_data, preprocess_point_cloud
from hardware.robot.TCP_client import send_pose, socket_init
from scipy.spatial.transform import Rotation
from hardware.camera.kinect import kinect4

voxel_size = 4

# 读取模型
# 读取文件夹下所有模型
def read_models(folder):
    models = []
    file_list = for_files_in_folder(folder, file_name_extension_filter=['.ply'])
    for file in file_list:
        models.append(prepare_data(file))
    return models


# 读取抓取位置
def read_grab_poses(folder):
    poses = []
    file_list = for_files_in_folder(folder, file_name_extension_filter=['.npy'])
    for file in file_list:
        poses.append(np.load(file))
    return poses

class estimation_data:
    def __init__(self, model, scene, label):
        self.model = model
        self.scene = scene
        self.label = label
        self.estimation_result = None

lable_2_name ={
    0: "0_duck",
    1: "1_pot",
    2: "2_cup",
    3: "3_drill"
}
name_2_lable = {
    "duck": 0,
    "pot": 1,
    "cup": 2,
    "drill": 3
}



def detect_and_registration(rgb_img, depth_img, d_cam_intrinsic, needed_name):
    # RGB识别
    boxes, scores, labels = detect(rgb_img, 'runtime')
    masks = []
    pose_estimator = []
    # pose_estimation_result_list = []

    # 根据RGB识别结果生成mask，扣深度图用于registration
    for box, score, label in zip(boxes[0], scores[0], labels[0]):
        if score < 0.5: continue
        mask = depth_img_copy_mask.generate_depth_mask(box, score, label, rgb_img.shape, depth_img.shape)
        masks.append(mask)
    for mask in masks:
        if mask.label != name_2_lable[needed_name]: continue
        depth_img_mask = depth_img_copy_mask.copy_img_mask(depth_img, mask)
        # cv2.imshow("detection result：", mask.maskimg)
        # cv2.imshow("origin", rgb_img)
        # cv2.waitKey(0)
        # print("this is ", mask.label)
        pc = depth_2_pc(depth_img_mask, d_cam_intrinsic)
        scene = pc, preprocess_point_cloud(pc, voxel_size, True)[0], preprocess_point_cloud(pc, voxel_size, True)[1]
        pose_estimator.append(estimation_data(models[mask.label], scene, mask.label))

    # registration位姿估计
    for this_estimation in pose_estimator:
        print("pose estimating:" + lable_2_name[this_estimation.label])
        # print("camera direction file folder is: data/model/%s/cam_dirs" % this_estimation.label)
        pose = run_registration(this_estimation.model, this_estimation.scene, voxel_size=voxel_size)
        this_estimation.estimation_result = pose
        # pose_estimation_result_list.append(pose)
    return pose_estimator


# 用图片系统测试
def system_test_using_image():
    use_real_robot = False
    if use_real_robot:
        s = socket_init()
    global models
    models = read_models("./data/model/BOP")
    grab_poses = read_grab_poses("./data/model/BOP")
    rgb_img_dir = "data/scene/scene/rgb/000000.png"
    d_img_dir = "data/scene/scene/depth/000000.png"
    rgb_img = cv2.imread(rgb_img_dir)
    depth_img = cv2.imread(d_img_dir, cv2.IMREAD_ANYDEPTH)
    
    BOP_cameraMtx = np.array([572.4114, 0.0, 325.2611, 0.0, 573.57043, 242.04899, 0.0, 0.0, 1.0], dtype=float).reshape(3, 3)
    pose_estimation_list = detect_and_registration(rgb_img, depth_img, BOP_cameraMtx, 'duck')
    
    for one_pose in pose_estimation_list:
        # 计算、显示结果
        transformation = one_pose.estimation_result.transformation
        model_trans = one_pose.model[0].transform(transformation)
        # scene = one_pose.scene[0]
        scene = depth_2_pc(depth_img, BOP_cameraMtx)
        scene.paint_uniform_color([0, 1, 0])
        model_trans.paint_uniform_color([1, 0, 0])

        # 画出夹爪坐标系
        d = np.linalg.norm(np.asarray(model_trans.get_max_bound()) - np.asarray(model_trans.get_min_bound()))
        grasp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=d / 2)
        grasp_frame.transform(transformation)
        o3d.visualization.draw_geometries([model_trans, scene, grasp_frame])
        
        if use_real_robot:
            t = transformation[0:3, 3].ravel()
            R = Rotation.from_matrix(transformation[0:3, 0:3])
            quat = R.as_quat()
            message = str(np.hstack((t, quat)))
            # 仅保留数字和空格
            message = ''.join(filter(lambda ch: ch in '0123456789e.+- ', message))
            print("send message:", message)
            if use_real_robot:
                send_pose(s, message)


def run_system():
    use_real_robot = False
    if use_real_robot:
        s = socket_init()
    cam = kinect4()
    img_color, img_depth = cam.get_capture()
    cameramtx = cam.cameramtx
    
     = depth_2_pc(img_depth, cameramtx)
    while 1:
        print("*")
        img_color, img_depth = cam.get_capture()
        pc_colored.points = depth_2_colored_pc(img_depth, img_color, cameramtx).points
        pc_colored.colors = depth_2_colored_pc(img_depth, img_color, cameramtx).colors
        vis.update_geometry(pc_colored)
        vis.poll_events()
        vis.update_renderer()
    
if __name__ == '__main__':
    system_test_using_image()
    # run_system()
    