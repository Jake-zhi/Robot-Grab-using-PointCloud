#!/usr/bin/python3
# coding=utf-8
'''
两种显示工具，open3d有bug但是需要在选视角时用它
plt可用，但需要程序设置坐标轴范围相等
'''

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # plt需要,不可删除


# 点云格式转numpy
def pcd_to_np(pcd): return np.asarray(pcd.points)


# numpy转点云格式
def np_to_pcd(pc):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    return pcd


def open3d_visualize(pc_list):
    try:
        o3d.visualization.draw_geometries(pc_list)
    except RuntimeError:
        print("o3d bug. Setting GPU to NVIDIA's graphics card may work.")
        open3d_visualize(pc_list)


def open3d_visualize_with_editing(pc_list):
    try:
        o3d.visualization.draw_geometries_with_editing(pc_list)
    except RuntimeError:
        print("o3d bug. Setting GPU to NVIDIA's graphics card may work.")
        open3d_visualize_with_editing(pc_list)

def plt_visualize(pc_list):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    x_center, y_center, z_center = 0, 0, 0
    for pc in pc_list:
        object_np = pcd_to_np(pc)
        x = object_np[:, 0]
        y = object_np[:, 1]
        z = object_np[:, 2]
        x_center = (x_center + np.average(x)) / 2
        y_center = (y_center + np.average(y)) / 2
        z_center = (z_center + np.average(z)) / 2
        ax.plot(x, y, z, '.', markersize=0.5)

    ax.set_xlabel('X')
    ax.set_xlim3d(x_center - 200, x_center + 200)
    ax.set_ylabel('Y')
    ax.set_ylim3d(y_center - 200, y_center + 200)
    ax.set_zlabel('Z')
    ax.set_zlim3d(z_center - 200, z_center + 200)
    plt.show()
