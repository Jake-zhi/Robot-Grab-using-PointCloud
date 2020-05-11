#!/usr/bin/python3
# coding=utf-8

import os

def for_files_in_folder(rootdir, file_name_extension_filter=[], dir_name_filter=[]): # 设置过滤后的文件类型 例如 filter = [".png"]
    result = []  # 所有的文件
    for maindir, subdir, file_name_list in os.walk(rootdir):
        # print("1:",maindir) #当前主目录
        # print("2:",subdir) #当前主目录下的所有目录
        # print("3:",file_name_list)  #当前主目录下的所有文件

        # 仅挑选感兴趣的文件夹
        skip = False
        for name_needed in dir_name_filter:
            if maindir.find(name_needed) == -1:
                skip = True   # 文件夹不包含所需关键字，跳过
        if skip:
            continue    # 不是想找的文件夹，以下跳过
        # 如果是感兴趣的文件夹
        for filename in file_name_list:
            apath = os.path.join(maindir, filename)  # 合并成一个完整路径
            ext = os.path.splitext(apath)[1]  # 获取文件后缀 [0]获取的是除了文件名以外的内容

            if ext in file_name_extension_filter:
                result.append(apath)
    return result

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

import open3d as o3d
folder = "D:\CODE&DATA\CODE\\robot_grasp\data\model\combined_model\down_sample"
file_list = for_files_in_folder(folder, file_name_extension_filter=['.ply'])
voxel_size = 1
for filename in file_list:
    print(filename)
    pc = o3d.io.read_point_cloud(filename)
    pc_down = pc.voxel_down_sample(voxel_size)
    o3d.io.write_point_cloud(filename, pc_down)

