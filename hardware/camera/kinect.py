from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import pyk4a
from pyk4a import Config, PyK4A, ColorResolution
import numpy as np
import cv2
import time


class kinect2:
    def __init__(self):
        self.cam = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Infrared | PyKinectV2.FrameSourceTypes_Depth)
        self.COLOR_HGT = self.cam.color_frame_desc.Height
        self.COLOR_WID = self.cam.color_frame_desc.Width
        self.DEPTH_HGT = self.cam.depth_frame_desc.Height
        self.DEPTH_WID = self.cam.depth_frame_desc.Width
        
        
    def read_color_frame(self):
        frame = self.cam.get_last_color_frame()
        frame_new = frame.reshape([self.COLOR_HGT, self.COLOR_WID, 4])[:, :, 0:3]
        return frame_new
    def read_depth_frame(self):
        frame = self.cam.get_last_depth_frame()
        frame_new = frame.reshape([self.DEPTH_HGT, self.DEPTH_WID])
        return frame_new
    def read_gray_frame(self):
        frame = self.cam.get_last_infrared_frame()
        frame_new = frame.reshape([self.DEPTH_HGT, self.DEPTH_WID])
        return frame_new


class kinect4:
    def __init__(self):
        print("使用pyk4a打开Kinect4")
        self.cam = PyK4A(Config(color_resolution=ColorResolution.RES_1536P,
                   depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
                   synchronized_images_only=True))
        self.COLOR_HGT = 1536
        self.COLOR_WID = 2048
        self.DEPTH_HGT = 576
        self.DEPTH_WID = 640
        self.distCoeffs = np.array([0.513059, -2.77779, -0.000323, 0.000703, 1.62693, 0.391017, -2.593868, 1.548565])
        self.cameramtx = np.array([[976.405945, 0, 1020.967651],
                                    [0, 976.266479, 779.519653],
                                    [0, 0, 1]])
        self.cam.connect()
        
        
    def get_capture(self):
        img_color, img_depth = self.cam.get_capture(transform_depth_to_color=True)  # Would also fetch the depth image
        if np.any(img_color) and np.any(img_depth):
            return img_color[:, :, :3], img_depth
    
    def __del__(self):
        self.cam.disconnect()


class kinect4_simulated:
    def __init__(self):
        print("假的Kinect4")
        self.COLOR_HGT = 1536
        self.COLOR_WID = 2048
        self.DEPTH_HGT = 576
        self.DEPTH_WID = 640
        self.distCoeffs = np.array([0.513059, -2.77779, -0.000323, 0.000703, 1.62693, 0.391017, -2.593868, 1.548565])
        self.cameramtx = np.array([[976.405945, 0, 1020.967651],
                                   [0, 976.266479, 779.519653],
                                   [0, 0, 1]])
    
    def get_capture(self):
        color_filename, depth_filename = "D:/CODE&DATA/CODE/robot_grasp/data/scene/MYDATA/rgb/0.png", "D:/CODE&DATA/CODE/robot_grasp/data/scene/MYDATA/depth/0.png"
        img_color = cv2.imread(color_filename)
        img_depth = cv2.imread(depth_filename, cv2.IMREAD_ANYDEPTH)
        return img_color, img_depth


class BOP_simulated:
    def __init__(self):
        print("模拟BOP相机")
        self.COLOR_HGT = 480
        self.COLOR_WID = 640
        self.DEPTH_HGT = 480
        self.DEPTH_WID = 640
        self.distCoeffs = np.array([0, 0, 0, 0, 0])
        self.cameramtx = np.array([572.4114, 0.0, 325.2611, 0.0, 573.57043, 242.04899, 0.0, 0.0, 1.0], dtype=float).reshape(3, 3)
    
    def get_capture(self):
        color_filename, depth_filename = "data/scene/BOP/rgb/000000.png", "data/scene/BOP/depth/000000.png"
        img_color = cv2.imread(color_filename)
        img_depth = cv2.imread(depth_filename, cv2.IMREAD_ANYDEPTH)
        return img_color, img_depth
    
def read_save_kinect2():
    root = "../data/kinect2/"
    cam = kinect2()
    cnt = 0
    skip = 20  # 跳过20帧
    while 1:
        # start = time.time()
        img_color = cam.read_color_frame()
        gray = cam.read_gray_frame()
        img_depth = cam.read_depth_frame()
        # print("FPS:", 1/(time.time()-start))
        cv2.imshow("img_color", img_color)
        cv2.imshow("gray", gray)
        key = cv2.waitKey(500)
        skip -= 1
        if key == 115 and skip <= 0:  # s键
            filename = root + "rgb/" + str(cnt) + ".png"
            print("writing file: ", filename)
            cv2.imwrite(filename, img_color)
            filename = root + "gray/" + str(cnt) + ".png"
            print("writing file: ", filename)
            cv2.imwrite(filename, gray)
            filename = root + "depth/" + str(cnt) + ".png"
            print("writing file: ", filename)
            cv2.imwrite(filename, img_depth)
            cnt += 1

def read_save_kinect4():
    root = "../data/kinect4/"
    cam = kinect4()
    cnt = 0
    skip = 5  # 跳过20帧
    while 1:
        img_color, img_depth = cam.get_capture()
        img_depth_show = img_depth.copy()
        img_depth_show -= np.amin(img_depth_show)
        img_depth_show = img_depth_show / np.amax(img_depth_show)
        img_depth_show *= 255
        cv2.imshow("ir", img_depth_show)
        cv2.imshow("img_color", img_color)

        key = cv2.waitKey(10)
        skip -= 1
        if key == 115 and skip <= 0:  # s键
            filename = root + "rgb/" + str(cnt) + ".png"
            print("writing file: ", filename)
            cv2.imwrite(filename, img_color)
            filename = root + "depth/" + str(cnt) + ".png"
            print("writing file: ", filename)
            cv2.imwrite(filename, img_depth)
            cnt += 1

def undistort_test():
    cam = kinect4()
    img_color, img_depth = cam.get_capture()
    
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam.cameramtx, cam.distCoeffs, img_color.shape[1::-1], 1)
    map1, map2 = cv2.initUndistortRectifyMap(cam.cameramtx, cam.distCoeffs, None, newcameramtx,
                                             img_color.shape[1::-1], cv2.CV_32FC1)
    cameramtx = newcameramtx
    
    while 1:
        start = time.time()
        img_color, img_depth = cam.get_capture()
        print("read frame FPS: ", 1/(time.time() - start))
        img_depth_show = img_depth.copy()
        img_depth_show -= np.amin(img_depth_show)
        img_depth_show = img_depth_show / np.amax(img_depth_show)
        img_depth_show *= 255
        cv2.imshow("ir", img_depth_show)
        cv2.imshow("img_color", img_color)
        start = time.time()
        img_color_undistort = cv2.remap(img_color, map1, map2, cv2.INTER_NEAREST)
        print("cost time:", time.time()-start)
        cv2.imshow("img_color_undistort", img_color_undistort)
        cv2.waitKey(1)
    
    
if __name__ == "__main__":
    undistort_test()
