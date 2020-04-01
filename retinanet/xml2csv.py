"""
运行方式：命令行 python xml2csv.py -i indir(图片及标注的母目录)
      注：必须参数: -i 指定包含有图片及标注的母文件夹，图片及标注可不在同一子目录里，但名称必须一一对应
                     (图片格式默认.jpg,若为其他格式可见代码中注释自行修改)
          可选参数: -p 交叉验证集拆分比，默认0.05
                   -t 生成训练集CSV文件名称，默认train.csv
                   -v 生成交叉验证集CSV文件名称，默认val.csv
                   -c 生成类别映射CSV文件名称，默认class.csv
"""

import os
import xml.etree.ElementTree as ET
import random
import math
import argparse


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--indir', type=str)
    parser.add_argument('-p', '--percent', type=float, default=0.05)
    parser.add_argument('-t', '--train', type=str, default='train.csv')
    parser.add_argument('-v', '--val', type=str, default='val.csv')
    parser.add_argument('-c', '--classes', type=str, default='class.csv')
    args = parser.parse_args()
    return args

#获取特定后缀名的文件列表
def get_file_index(indir, postfix):
    file_list = []
    for root, dirs, files in os.walk(indir):
        for name in files:
            if postfix in name:
                file_list.append(os.path.join(root, name))
    return file_list

#写入标注信息
def convert_annotation(csv, address_list):
    cls_list = []
    with open(csv, 'w') as f:
        for i, address in enumerate(address_list):
            in_file = open(address, encoding='utf8')
            strXml =in_file.read()
            in_file.close()
            root=ET.XML(strXml)
            for obj in root.iter('object'):
                cls = obj.find('name').text
                cls_list.append(cls)
                xmlbox = obj.find('bndbox')
                b = (int(xmlbox.find('xmin').text), int(xmlbox.find('ymin').text),
                     int(xmlbox.find('xmax').text), int(xmlbox.find('ymax').text))
                f.write(file_dict[address_list[i]])
                f.write( "," + ",".join([str(a) for a in b]) + ',' + cls)
                f.write('\n')
    return cls_list


if __name__ == "__main__":
    args = parse_args()
    file_address = args.indir
    test_percent = args.percent
    train_csv = args.train
    test_csv = args.val
    class_csv = args.classes
    Annotations = get_file_index(file_address, '.xml')
    Annotations.sort()
    JPEGfiles = get_file_index(file_address, '.png') #可根据自己数据集图片后缀名修改
    JPEGfiles.sort()
    assert len(Annotations) == len(JPEGfiles) #若XML文件和图片文件名不能一一对应即报错
    file_dict = dict(zip(Annotations, JPEGfiles))
    num = len(Annotations)
    test = random.sample(k=math.ceil(num*test_percent), population=Annotations)
    train = list(set(Annotations) - set(test))

    cls_list1 = convert_annotation(train_csv, train)
    cls_list2 = convert_annotation(test_csv, test)
    cls_unique = list(set(cls_list1+cls_list2))

    with open(class_csv, 'w') as f:
        for i, cls in enumerate(cls_unique):
            f.write(cls + ',' + str(i) + '\n')