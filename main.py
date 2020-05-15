from RGB_D import depth_2_pc, depth_2_colored_pc, generate_fast_calculate_table
from pos_estimation.registration import run_registration
from process_data import depth_img_copy_mask
import cv2
import numpy as np
from process_data.about_os import for_files_in_folder
import open3d as o3d
from process_data.point_cloud_preprocess import prepare_data, preprocess_point_cloud
from hardware.robot.TCP_client import send_pose, socket_init
from scipy.spatial.transform import Rotation
from hardware.camera.kinect import kinect4, BOP_simulated
import multiprocessing
from multiprocessing.managers import BaseManager
import threading
import time


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

# 等待位姿估计的队列
class detect_result_buffer:
    def __init__(self):
        # 记录六个检测结果 便于将来运动跟踪
        self.results = []
        self.timestamps = []
    def append(self, newone, timestamp):
        self.results.append(newone)
        self.timestamps.append(time.time())
    def pop(self):
        return self.results.pop(0), self.timestamps.pop(0)
 
class img_buff:
    def __init__(self, camera_mtx, img_shape):
        self.data = []
        self.camera_mtx = camera_mtx
        self.img_shape = img_shape
    def append(self, newimg):
        if len(self.data) >= 6:
            self.data.pop(0)
            self.data.append(newimg)
        else:
            self.data.append(newimg)
    def pop(self, num=0):
        if len(self.data) > 1:
            return self.data.pop(num)
    def get_camera_mtx(self):
        return self.camera_mtx


# RGB检测，将结果加入待估计的list中
def detect_add_2_logger(cam, detect_result_buffer, img_color_buff, img_depth_buff):
    from retinanet.detection import detect_init, detect
    dnn_model = detect_init()
    while 1:
        img_color, img_depth = cam.get_capture()
        timestamp =time.time()
            
        # RGB识别
        boxes, scores, labels = detect(img_color, dnn_model, 'debug')
        results = []
    
        # 根据RGB识别结果生成mask，扣深度图用于registration
        for box, score, label in zip(boxes[0], scores[0], labels[0]):
            if score < 0.8: continue
            mask = depth_img_copy_mask.generate_depth_mask(box, img_color.shape, img_depth.shape)
            result = [mask, score, label]
            results.append(result)
        global share_lock
        share_lock.acquire()
        detect_result_buffer.append(results, timestamp)
        img_color_buff.append(img_color)
        img_depth_buff.append(img_depth)
        share_lock.release()
            
# 对logger里的数据们位姿估计
def registrate_pose_estimation_queue(pose_estimation_queue, img_depth_buff):
    global voxel_size
    global models
    global share_lock
    global label_2_name
    fast_calculate_table = generate_fast_calculate_table(img_depth_buff.img_shape, cameramtx)
    while 1:
        share_lock.acquire()
        img_depth = img_depth_buff.pop(0)
        img_color = img_color_buff.pop(0)
        share_lock.release()
        if img_depth is not None:
            pose_estimate_list = []
            for mask in pose_estimation_queue.latest():
                depth_img_mask = depth_img_copy_mask.copy_img_mask(img_depth, mask)
                pc = depth_2_pc(depth_img_mask, fast_calculate_table)
                scene = pc, preprocess_point_cloud(pc, voxel_size, True)[0], preprocess_point_cloud(pc, voxel_size, True)[1]
                pose_estimate_list.append(estimation_data(models[mask.label], scene, mask.label))
            for this_estimation in pose_estimate_list:
                print("pose estimating:" + label_2_name[this_estimation.label])
                pose = run_registration(this_estimation.model, this_estimation.scene, voxel_size=voxel_size)
                this_estimation.estimation_result = pose
            for one_pose in pose_estimate_list:
                # 计算、显示结果
                transformation = one_pose.estimation_result.transformation
                model_trans = one_pose.model[0].transform(transformation)
                # scene = one_pose.scene[0]
                scene = depth_2_pc(img_depth, fast_calculate_table)
                scene.paint_uniform_color([0, 1, 0])
                model_trans.paint_uniform_color([1, 0, 0])

                # 画出夹爪坐标系
                d = np.linalg.norm(
                    np.asarray(model_trans.get_max_bound()) - np.asarray(model_trans.get_min_bound()))
                grasp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=d / 2)
                grasp_frame.transform(transformation)
                o3d.visualization.draw_geometries([model_trans, scene, grasp_frame])
    

# 用图片系统测试
def system_test_using_image():
    use_real_robot = False
    if use_real_robot:
        s = socket_init()
    
    grab_poses = read_grab_poses("./data/model/BOP")
    rgb_img_dir = "data/scene/BOP/rgb/000000.png"
    d_img_dir = "data/scene/BOP/depth/000000.png"
    Tracker = detect_result_buffer()
    rgb_img = cv2.imread(rgb_img_dir)
    depth_img = cv2.imread(d_img_dir, cv2.IMREAD_ANYDEPTH)
    
    BOP_cameraMtx = np.array([572.4114, 0.0, 325.2611, 0.0, 573.57043, 242.04899, 0.0, 0.0, 1.0], dtype=float).reshape(3, 3)
    pose_estimation_list = detect_add_2_logger(rgb_img, depth_img, BOP_cameraMtx)
    Tracker.append(pose_estimation_list)
    pose_estimation_list = registrate_pose_estimation_queue(Tracker.latest())

    fast_calculate_table = generate_fast_calculate_table(depth_img.shape(), BOP_cameraMtx)
    for one_pose in pose_estimation_list:
        # 计算、显示结果
        transformation = one_pose.estimation_result.transformation
        model_trans = one_pose.model[0].transform(transformation)
        # scene = one_pose.scene[0]
        scene = depth_2_pc(depth_img, fast_calculate_table)
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



share_lock = threading.Lock()
voxel_size = 4
label_2_name = {0: 'duck', 1: 'pot', 2: 'cup', 3: 'drill'}
# label_2_name = {0: 'M1', 1: 'M3', 2: 'M4', 3: 'M7', 4: 'M8', 5: 'M5', 6: 'M6', 7: 'M2'}
name_2_lable = {"duck": 0, "pot": 1, "cup": 2, "drill": 3}
# models = read_models("./data/model/using")
models = read_models("./data/model/BOP")
if __name__ == '__main__':
    simulate = True
    if not simulate:
        cam = kinect4()
        cameramtx = cam.cameramtx
        img_color_shape = (cam.COLOR_HGT, cam.COLOR_WID, 3)
        img_depth_shape = (cam.DEPTH_HGT, cam.DEPTH_WID)
    else:
        cam = BOP_simulated()
        cameramtx = cam.cameramtx
        img_color_shape = (cam.COLOR_HGT, cam.COLOR_WID, 3)
        img_depth_shape = (cam.DEPTH_HGT, cam.DEPTH_WID)

    
    # system_test_using_image()
    # run_system()
    manager = BaseManager()
    manager.register("pose_estimation_queue", detect_result_buffer)
    manager.register("img_color_buff", img_buff)
    manager.register("img_depth_buff", img_buff)
    manager.start()
    global_pose_estimation_queue = manager.pose_estimation_queue()
    img_color_buff = manager.img_color_buff(cameramtx, img_color_shape)
    img_depth_buff = manager.img_depth_buff(cameramtx, img_depth_shape)
    
    # load label to names mapping for visualization purposes
    
    process_list = []
    # 创建进程 RGB检测
    tmp_process = multiprocessing.Process(target=detect_add_2_logger, args=[cam, global_pose_estimation_queue, img_color_buff, img_depth_buff])
    process_list.append(tmp_process)
    # 创建进程 位姿估计
    tmp_process = multiprocessing.Process(target=registrate_pose_estimation_queue, args=[global_pose_estimation_queue, img_depth_buff])
    process_list.append(tmp_process)
    # 启动所有进程
    for process in process_list:
        process.start()
    for process in process_list:
        process.join()

