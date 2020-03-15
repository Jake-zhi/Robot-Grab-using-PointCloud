# Robot-Grab-using-PointCloud
主要使用open3d，修改了registration的ransac部分源码

本人毕设
使用RGBD相机的机器抓取，陆续更新

# BUG
1. 可视化时遇到RunTimeError：GLFW Error，open3d github issue里有人试了
设置python使用n卡，可解决报错卡退问题。
2. 编译open3d源码时cmake error：缺失pybind11Target.cmake，手动编译pybind11源码,
将编译得到的此文件复制到对应位置即可暂时解决问题，根治问题应该需要修改cmake生成文件相关代码。

## open3d源码修改：

1. ransac跳出条件，当达到一定fitness和rmse时跳出
2. class RegistrationResult() 默认构造函数中修改fitness和rmse缺省值