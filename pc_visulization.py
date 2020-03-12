#!/usr/bin/python3
# coding=utf-8
'''
两种显示工具，open3d有bug但是需要在选视角时用它
plt可用，但需要程序设置坐标轴范围相等
'''

#!/usr/bin/python3
# coding=utf-8
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D     # plt需要,不可删除
from point_cloud_preprocess import prepare_data, pcd_to_np


def open3d_visualize(pc_list):
    try:
        o3d.visualization.draw_geometries(pc_list)
    except RuntimeError:
        print("o3d bug.")
        open3d_visualize(pc_list)


def plt_visualize(pc_list):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    x_center, y_center, z_center = 0, 0, 0
    for pc in pc_list:
        object_np = pcd_to_np(pc)
        x = object_np[:, 0]
        y = object_np[:, 1]
        z = object_np[:, 2]
        x_center = (x_center+np.average(x))/2
        y_center = (y_center+np.average(y))/2
        z_center = (z_center+np.average(z))/2
        ax.plot(x, y, z, '.', markersize=0.5)

    ax.set_xlabel('X')
    ax.set_xlim3d(x_center-200, x_center+200)
    ax.set_ylabel('Y')
    ax.set_ylim3d(y_center-200, y_center+200)
    ax.set_zlabel('Z')
    ax.set_zlim3d(z_center-200, z_center+200)
    plt.show()


def prepare_model_and_open3d_visualize(filename, voxel_size=4):
    src_pc, _, _ = prepare_data(filename, voxel_size=voxel_size)
    open3d_visualize([src_pc])

def prepare_model_and_plt_visualize(filename, voxel_size = 4):
    src_pc, _, _ = prepare_data(filename, voxel_size=voxel_size)
    plt_visualize([src_pc])

if __name__ == "__main__":
    # prepare_model_and_plt_visualize("data/model_duck.ply")
    prepare_model_and_open3d_visualize("data/model_duck.ply")