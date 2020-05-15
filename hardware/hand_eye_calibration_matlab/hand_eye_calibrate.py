import scipy.io as io
import numpy as np
from scipy.spatial.transform import Rotation

# matlab存储方式和正常方式转换
def format_Mat2np(mat):
    shape = mat.shape[::-1]
    newdata = np.zeros(shape)
    for i in range(shape[0]):
        newdata[i] = mat[:, :, i]
    return newdata
def format_np2Mat(ndarray):
    shape = ndarray.shape[::-1]
    newdata = np.zeros(shape)
    for i in range(shape[-1]):
        newdata[:, :, i] = ndarray[i]
    return newdata
    

def read_convert_test(filename="Example data/armMat.mat", key="armMat"):
    data = io.loadmat(filename)
    mat = data[key]
    
    print("***************** Convert mat to ndarray ******************************")
    ndarray = format_Mat2np(mat)
    print(ndarray)
    print("***************** Convert ndarray to mat ******************************")
    mat = format_np2Mat(ndarray)
    print(mat)


# 输入变换矩阵们，nx4x4, 保存为mat
def write_transformations_as_mat(ndarray, filename="armMat.mat"):
    savedict = {"armMat": format_np2Mat(ndarray)}
    io.savemat(filename, savedict)


# 将记录的末端位置转换为手眼标定需要的mat
def convert_poselog_to_mat(fin_name, mat_fout_name):
    fin = open(fin_name, "r")
    lines = fin.readlines()
    poses = []
    for line in lines:
        line = ''.join(filter(lambda ch: ch in '0123456789e.+- \t', line))
        # x, y, z, quat1, quat2, quat3, quat4 = line.split()
        
        x, y, z, rx, ry, rz = line.split()
        t = np.array([[float(x)], [float(y)], [float(z)]])
        # R = Rotation.from_quat([quat1, quat2, quat3, quat4])
        R = Rotation.from_rotvec([float(rx), float(ry), float(rz)])
        R_mtx = R.as_matrix()
        transformation = np.hstack((R_mtx, t))
        transformation = np.vstack((transformation, np.array([0, 0, 0, 1])))
        poses.append(transformation)
    ndarray = np.array(poses)
    mat = format_np2Mat(ndarray)
    io.savemat(mat_fout_name, {"armMat": mat})
        


if __name__ == '__main__':
    
    convert_poselog_to_mat("My Data/poselog.txt", "My Data/armMat.mat")

    # mat = io.loadmat("/armMat.mat")
    # io.savemat("Example data/TestArmMat.mat", mat)
    # read_convert_test("armMat.mat")