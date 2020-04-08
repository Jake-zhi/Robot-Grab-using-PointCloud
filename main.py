import sys
sys.path.append("../depth2pc")
import convert_depth_img
from pos_estimation.registration import run_registration
from process_data import depth_img_copy_mask
from retinanet.detection import detect
import cv2
from process_data.about_os import for_files_in_folder
import open3d as o3d
from process_data.point_cloud_preprocess import prepare_data, preprocess_point_cloud
voxel_size = 4


#读取模型
#读取文件夹下所有模型
def read_models(folder):
    models = []
    file_list = for_files_in_folder(folder, file_name_extension_filter=['.ply'])
    for file in file_list:
        models.append(prepare_data(file))
    return models


class estimation_data:
    def __init__(self, model, scene, label):
        self.model = model
        self.scene = scene
        self.label = label

classes ={
    0: "0_duck",
    1: "1_pot",
    2: "2_cup",
    3: "3_drill"
}

models = read_models("./data/model")

rgb_img_dir = "data/scene/scene/rgb/000000.png"
d_img_dir = "data/scene/scene/depth/000000.png"
rgb_img = cv2.imread(rgb_img_dir)
depth_img = cv2.imread(d_img_dir, cv2.IMREAD_ANYDEPTH)

def main():
    # RGB识别
    boxes, scores, labels = detect(rgb_img, 'debug')
    masks = []
    pose_estimation_list = []
    pose_estimation_result_list = []

    # 根据RGB识别结果生成mask，扣深度图用于registration
    for box, score, label in zip(boxes[0], scores[0], labels[0]):
        if score < 0.5: continue
        mask = depth_img_copy_mask.generate_depth_mask(box, score, label, rgb_img.shape, depth_img.shape)
        masks.append(mask)
    for mask in masks:
        depth_img_mask = depth_img_copy_mask.copy_img_mask(depth_img, mask)
        # cv2.imshow("detection result：", mask.maskimg)
        # cv2.imshow("origin", rgb_img)
        # cv2.waitKey(0)
        # print("this is ", mask.label)
        pc = convert_depth_img.depth2pc(depth_img_mask)
        scene = pc, preprocess_point_cloud(pc, voxel_size, True)[0], preprocess_point_cloud(pc, voxel_size, True)[1]
        pose_estimation_list.append(estimation_data(models[mask.label], scene, mask.label))

    # registration位姿估计
    for this_estimation in pose_estimation_list:
        print("rgb_d_to_pointcloud:" + classes[this_estimation.label])
        # print("camera direction file folder is: data/model/%s/cam_dirs" % this_estimation.label)
        # run_registration(this_estimation.model, this_estimation.scene, direction_file_dir=("data/model/%s/cam_dirs"%this_estimation.label), voxel_size=voxel_size)
        pose = run_registration(this_estimation.model, this_estimation.scene, voxel_size=voxel_size)
        pose_estimation_result_list.append(pose)



if __name__ == '__main__':
    main()
