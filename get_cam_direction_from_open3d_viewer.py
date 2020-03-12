#!/usr/bin/python3
# coding=utf-8
'''
开启两个进程，open3d的点云显示 和 读取json
在open3d界面中调整点云至合适位置，按下crtl+c
粘贴至弹出的文本文件中保存，关闭文本文件
解析json并存储相机方向至文件
关闭open3d界面

注意：请根据实际弹出的文本编辑器改变editor，避免误关程序

'''

# TODO
## 根据front, up, origin, 计算相机观察位置
## 文件存储观察位置(注意增量式)

editor = "notepad.exe"                  # 系统默认文本编辑器(根据实际情况选择)
filename = "data/model_duck.ply"        # 模型文件
voxel_size = 5                          # 模型体素降采样大小(与主程序一致)

from pc_visulization import prepare_model_and_open3d_visualize
import json
import os
from multiprocessing import Process
import sys
import numpy as np
import time


# 解析信息返回front方向和up方向
# 粘贴json内容至文本文件
# return front,up
def save_cam_dir_from_json(out_filename):
    fin = open(".\data\\temp\\trajectory.txt", encoding='utf-8', mode='w')
    fin.write('请在弹出的open3d显示界面调整物体姿态，调整到满意位置时按下ctrl+c，在本文件中全选，粘贴保存并退出。')
    fin.close()
    os.system(".\data\\temp\\trajectory.txt")
    try:
        os.rename(".\data\\temp\\trajectory.txt", ".\data\\temp\\trajectory.json")
    except OSError:
        os.remove(".\data\\temp\\trajectory.json")
        os.rename(".\data\\temp\\trajectory.txt", ".\data\\temp\\trajectory.json")

    try:
        fin = open("./data/temp/trajectory.json", encoding='utf-8')
        user_dic = json.load(fin)
        trajectory = user_dic['trajectory'][0]
        cam_dir = trajectory['front']
        '''
        有被自己蠢到，下方代码可用于计算点云之间的变换矩阵，在此处并没有什么卵用
        origin = np.array([0, 0, 0])
        front = np.array(trajectory['front'])
        up = np.array(trajectory['up'])
        points_np = np.vstack((origin, front, up))
        origin_frame = np_to_pcd(np.array([ [0, 0, 0],
                                            [0, 0, 1],
                                            [0, 1, 0]]))
        new_frame = np_to_pcd(points_np)
        corrs = o3d.utility.Vector2iVector(np.array([[0, 0], [1, 1], [2, 2]]))
        estimator = o3d.registration.TransformationEstimationPointToPoint()
        R = estimator.compute_transformation(origin_frame, new_frame, corrs)[0:3, 0:3]
        cam_dir = np.dot(R, [[0], [0], [1]]).T
        np.savetxt(out_filename, cam_dir)
        '''
        np.savetxt(out_filename, cam_dir)
        return 0
    except json.decoder.JSONDecodeError:
        print("json文件格式错误,未保存相机角度")
        return -1
    except KeyError:
        print("json文件信息缺失,未保存相机角度")
        return -1


# 记录感兴趣的视角
def get_cam_directions():
    del_file_in_folder("data/cam_dirs")
    for i in range(100):
        result = save_cam_dir_from_json("data/cam_dirs/dir_%s.txt" % i)
        if result != 0:
            print("读取json时错误")


# 关闭其他应用程序
# pro_name:将要关闭的程序
def end_program(pro_name):
    os.system('%s%s' % ("taskkill /F /IM ", pro_name))


# 删除文件夹下所有文件和文件夹
def del_file_in_folder(path_data):
    for i in os.listdir(path_data) :# os.listdir(path_data)#返回一个列表，里面是当前目录下面的所有东西的相对路径
        file_data = path_data + "\\" + i#当前文件夹的下面的所有东西的绝对路径
        if os.path.isfile(file_data) == True:#os.path.isfile判断是否为文件,如果是文件,就删除.如果是文件夹.递归给del_file.
            os.remove(file_data)
        else:
            del_file_in_folder(file_data)
            os.rmdir(file_data)


def main():
    try:
        process_open3d_visualization = Process(target=prepare_model_and_open3d_visualize,
                                                       args=(filename, voxel_size))
        process_get_cam_directions = Process(target=get_cam_directions)
        process_get_cam_directions.start()
        time.sleep(3)
        process_open3d_visualization.start()
    except:
        raise ChildProcessError("无法启动进程。")
    while 1:
        if process_open3d_visualization.is_alive() is False:
            process_get_cam_directions.terminate()
            end_program(editor)
            break
    print("记录相机方向完成，退出。")

if __name__ == "__main__":
    main()
