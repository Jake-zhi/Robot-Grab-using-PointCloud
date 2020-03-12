#!/usr/bin/python3
# coding=utf-8

import os

# 返回文件夹下的文件目录
def for_files_in_dir(file_dir):
    file_list = []
    for name in os.listdir(file_dir) :       # os.listdir(path_data)#返回一个列表，里面是当前目录下面的所有东西的相对路径
        file_name = file_dir + "\\" + name   #当前文件夹的下面的所有东西的绝对路径
        file_list.append(file_name)
    return file_list


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

