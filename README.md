# ROS 下 Kinect v2 的驱动安装
<!-- TOC -->

- [ROS 下 Kinect v2 的驱动安装](#ros-下-kinect-v2-的驱动安装)
    - [TODO](#todo)
    - [安装libfreenect2](#安装libfreenect2)
    - [下载iai-kinect2](#下载iai-kinect2)
    - [标定(更改标定图片的大小)](#标定更改标定图片的大小)
    - [保存深度图像彩色图像点云](#保存深度图像彩色图像点云)

<!-- /TOC -->
##　TODO
- [x] kinect v2 在ROS环境中的使用配置
- [ ] 同步保存彩色图 深度图 点云图
- [ ] 深度图 彩色图 点云图的对齐
- [ ] 通过彩色图像的像素坐标获取深度图中的对应深度值,以及点云坐标

在ROS环境使用Kinectv2 主要依靠iai-kinect2.  
Github地址： https://github.com/code-iai/iai_kinect2
## 安装libfreenect2
1. 下载 libfreenect2 源码
    ```
    git clone https://github.com/OpenKinect/libfreenect2.git
    cd libfreenect2
    ```
2. 安装编译工具
    ```
    sudo apt-get install build-essential cmake pkg-config
    ```
3. 安装linusb
    ```
    sudo apt-get install libusb-1.0-0-dev
    ```
4. 安装TurboJPEG
    ```
    sudo apt-get install libturbojpeg libjpeg-turbo8-dev
    ```
5. 安装OpenGL
    ```
    sudo apt-get install libglfw3-dev
    ```
6. 安装OpenNI2
    ```
    sudo apt-get install libopenni2-dev
    ```
7. 编译构建
    ```
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/  #修改了原来的安装位置
    make
    sudo make install  #这里直接用root权限安装，如果是普通权限有的时候会出现安装不成功的情况
    ```
8. 设置udev规则
    ```
    sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
    ```
9. 将Kinect V2重新插拔，之后运行测试
    ```
    ./bin/Protonect  #重新接入之后运行测试
    ```
## 下载iai-kinect2
1. 从Github上面下载工程源码到工作空间内src
    ```
    cd ~/catkin_ws/src/
    git clone https://github.com/code-iai/iai_kinect2.git
    cd iai_kinect2
    rosdep install -r --from-paths .
    cd ~/catkin_ws
    catkin_make -DCMAKE_BUILD_TYPE="Release"
    ```
2. 启动 
    ```
    roslaunch kinect2_bridge kinect2_bridge.launch
    ```
3. 查看
    ```
    rosrun kinect2_viewer kinect2_viewer sd cloud
    ##或者
    rosrun rqt_image_view rqt_image_view 
    ```
## 标定(更改标定图片的大小)
kinect的数据是分成3类的，包括hd，qhd，sd。其中hd(1920*1080),qhd(960*540)，sd(480*270).  
默认标定程序使用的彩色图像大小是hd(1920*1080)，标定出的相机内参数矩阵只适合于1920*1080大小的图像.  
为了使用Kinect v2采集的qhd(960*540)、或sd(480*270)的彩色图像，需要相应的标定出的相机内参.
更改kinect2_calibration.cpp文件
```
std::string topicColor = "/" + ns + K2_TOPIC_HD + K2_TOPIC_IMAGE_MONO;
std::string topicIr = "/" + ns + K2_TOPIC_SD + K2_TOPIC_IMAGE_IR;
std::string topicDepth = "/" + ns + K2_TOPIC_SD + K2_TOPIC_IMAGE_DEPTH;
```
标定需要每个图片100张　至少两个距离　不同的orientation 和　在图片的不同位置
device serial记录一下
rosrun kinect2_calibration kinect2_calibration chess9x11x0.02 record color
按１００次空格
之后再重新编译
## 保存深度图像彩色图像点云

更改viewer.cpp文件
