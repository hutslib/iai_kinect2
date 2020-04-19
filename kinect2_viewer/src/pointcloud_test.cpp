// -------------------------------------------
//  @description: 产生骨架点云病将将所需的骨架点转换到世界坐标系中
//  @author: hts
//  @data: 2020-04-11
//  @version: wpdwp
// -------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#define pi 3.14159265359

class Posepoint
{
private:
  Vector3f Lshoulder;  
public:
void test()
{
  Lshoulder(0) = 1;
}
};
int main(int argc, char**argv)
{
    // Vector3f result;
    // result<<1,11,11;
    // Vector3f a, b,result;
    // a=Lshoulder-Pelv;
    // b=Rshoulder-Pelv;
    // cout<<a<<"  "<<b<<endl;
    // result = a.cross(b);
    // cout<<result<<endl;
    Eigen::Vector3f result(0, 0, 0);
	  result.x() = 1;
	  result[2] = 1;
	  AngleAxisd angle_axis3(pi / 4, Eigen::Vector3d(0, 1, 0));//1系绕y轴逆时针旋转45得到2系
	  Vector3f rotated_result = angle_axis3.matrix().cast<float>()*result;
	  cout << "绕y轴逆时针旋转45°(R12):" << endl << angle_axis3.matrix() << endl;//注意和绕x轴z轴不一样
	  cout << "(1, 0, 1)旋转后:" << endl << rotated_result.transpose() << endl;
}