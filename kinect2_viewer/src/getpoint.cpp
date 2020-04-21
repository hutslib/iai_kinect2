// -------------------------------------------
//  @description: 根据彩色图像的像素产生点云代码 产生三维骨架点云 计算外积 计算方差 
//  @author: hts
//  @data: 2020-04-20
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
  std::mutex lock;
  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PCDWriter writer;
  bool visualize_stop;
  double pick_points1[6];
  double pick_points2[18];
  Vector3f Lshoulder, Rshoulder, Pelv;
  Vector3f Head,Neck,Thrx,Lhip,Lknee,Lankle,Rhip,Rknee,Rankle;  
public:
  Posepoint():visualize_stop(false)
  {
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    // cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    static double my_points[6] = {255.67636108398438,292.16888427734375,320.4430236816406,266.26220703125,330.15802001953125,298.64556884765625};
    static double my_points2[18] = {325.0851135253906, 354.8621826171875, 277.11724853515625, 306.894287109375, 234.90553283691406, 281.9510192871094, 
313.57281494140625, 276.19488525390625, 317.4102478027344, 326.0814208984375, 244.4990997314453, 274.2761535644531, 213.79966735839844, 239.7393035888672, 213.79966735839844, 235.90187072753906, 
219.55581665039062, 216.7147216796875};
    for(int i=0; i<6; i++)
    {
      pick_points1[i] = my_points[i];
    }
    for(int j=0; j<18; j++)
    {
      pick_points2[j] = my_points2[j];
    }
  }
  ~Posepoint()
  {
  }
public:
    void start(std::string color_path, std::string depth_path)
  {
    cout<<"开始!"<<endl;
    color = cv::imread(color_path);
    depth = cv::imread(depth_path,2);
    // cv::imshow("color", color);
    // cv::imshow("depth", depth);
    // cv::waitKey(0);
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    readCameraInfo();
    createLookup(this->color.cols, this->color.rows); //像极坐标系中的值保存在lookupX　lookupY里面
    cloudViewer();
  }
  // 3x3内参矩阵
//   367.933   0         254.169
//     0     367.933     204.267
//     0       0            1
    void readCameraInfo()
  {
    double cameraInfoK[9]={367.933 , 0 , 254.169,
                           0 , 367.933 , 204.267,
                          0 , 0 , 1             }; 
    double *itC = cameraMatrixColor.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfoK[i];
      // cout<<*itC<<endl;
    }
    // cout<<cameraMatrixColor.at<double>(0, 0)<<endl;
  }
    // 求在相机坐标系中的坐标
    void createLookup(size_t width, size_t height)
  {
    // width是列->x　height是行->y
    // 得到相机的内参数
    //  成像模型
// [u       [ fx　0 cx   [x
//  v  = 1/z  0  fy cy    y
//  1]        0   0  1]   z]
// 求逆矩阵　得到像极坐标系中的坐标　
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;
    cout<<"相机的内参:"<<fx<<" "<<fy<<" "<<cx<<" "<<cy<<" "<<endl;
    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
      // cout<<*it<<endl;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
      // cout<<*it<<endl;
    }
  }
    void cloudViewer()
  {
    cv::Mat color, depth;
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";

    lock.lock();
    color = this->color;
    depth = this->depth;
    // updateCloud = false;
    lock.unlock();

    createCloud(depth, color, cloud);
    // 将点云数据添加到视窗中，并为其定义一个唯一的字符串作为ID号，利用此ID号保证其他成员方法也能表示该点云。
    // 多次调用addPointCloud()可以实现多个点云的叠加，每调用一次就创建一个新的ID号。如果想要更新一个已经
    // 显示的点云，用户必须先调用removePointCloud()，并提供新的ID号。（在PCL1.1版本之后直接调用updatePointCloud()
    //  就可以了，不必手动调用removePointCloud()就可实现点云更新）
    visualizer->addPointCloud(cloud, cloudName);
    // 修改现实点云的尺寸。用户可通过该方法控制点云在视窗中的显示方式
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    // 设置XYZ三个坐标轴的大小和长度，该值也可以缺省
    // 查看复杂的点云图像会让用户没有方向感，为了让用户保持正确的方向判断，需要显示坐标轴。三个坐标轴X（R，红色）
    // Y（G，绿色）Z（B，蓝色）分别用三种不同颜色的圆柱体代替
    visualizer->addCoordinateSystem(1.0);
    // 通过设置相机参数是用户从默认的角度和方向观察点
    visualizer->initCameraParameters();
    // 设置窗口viewer的背景颜色
    visualizer->setBackgroundColor(0, 0, 0);
    // visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
    visualizer->setPosition(0, 0);
    visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->registerKeyboardCallback(&Posepoint::keyboardEvent, *this);
    saveCloud(cloud);
    // visualizer->spinOnce(300);
    while (!visualize_stop) {
        visualizer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }

    visualizer->close();
  }
    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
  {
    int body_part;
    const float badPoint = std::numeric_limits<float>::quiet_NaN();
    // #pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
      // 创建点云row行，每一行有col列
      pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
      // ptr函数访问任意一行像素的首地址　
      const uint16_t *itD = depth.ptr<uint16_t>(r);
      const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
      const float y = lookupY.at<float>(0, r);
      const float *itX = lookupX.ptr<float>();

        for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
      {
        register const float depthValue = *itD / 1000.0f;
        // Check for invalid measurements
        if(*itD == 0)
        {
          // not valid
          itP->x = itP->y = itP->z = badPoint;
          itP->rgba = 0;
          continue;
        }
        itP->z = depthValue;
        itP->x = *itX * depthValue;
        itP->y = y * depthValue;
        itP->b = itC->val[0];
        itP->g = itC->val[1];
        itP->r = itC->val[2];
        itP->a = 255;
          bool picked = false;
          if(int(pick_points1[0]) == r && size_t(pick_points1[1]) == c)
        {
            cout<<"点("<<c<<","<<r<<")选中"<<endl;
            picked = true;
            body_part = 0;        
        }
          if(int(pick_points1[2]) == r && size_t(pick_points1[3]) == c)
        {
            cout<<"点("<<c<<","<<r<<")选中"<<endl;
            picked = true;
            body_part = 1;        
        }
          if(int(pick_points1[4]) == r && size_t(pick_points1[5]) == c)
        {
            cout<<"点("<<c<<","<<r<<")选中"<<endl;
            picked = true;
            body_part = 2;        
        }
          if(int(pick_points2[0]) == r && size_t(pick_points2[1]) == c)
        {
            cout<<"点("<<c<<","<<r<<")选中"<<endl;
            picked = true;
            body_part = 3;        
        }
          if(int(pick_points2[2]) == r && size_t(pick_points2[3]) == c)
        {
            cout<<"点("<<c<<","<<r<<")选中"<<endl;
            picked = true;
            body_part = 4;        
        }
          if(int(pick_points2[4]) == r && size_t(pick_points2[5]) == c)
        {
            cout<<"点("<<c<<","<<r<<")选中"<<endl;
            picked = true;
            body_part = 5;        
        }
          if(int(pick_points2[6]) == r && size_t(pick_points2[7]) == c)
        {
            cout<<"点("<<c<<","<<r<<")选中"<<endl;
            picked = true;
            body_part = 6;        
        }
          if(int(pick_points2[8]) == r && size_t(pick_points2[9]) == c)
        {
            cout<<"点("<<c<<","<<r<<")选中"<<endl;
            picked = true;
            body_part = 7;        
        }
          if(int(pick_points2[10]) == r && size_t(pick_points2[11]) == c)
        {
            cout<<"点("<<c<<","<<r<<")选中"<<endl;
            picked = true;
            body_part = 8;        
        }
        if (picked)
        {
          register const float depthValue = *itD / 1000.0f;
          switch(body_part)
          {
          case 0:;
                Lshoulder(0) = itP->x; Lshoulder(1) = itP->y; Lshoulder(2) = itP->z;
                break;
          case 1:
                Rshoulder(0) = itP->x; Rshoulder(1) = itP->y; Rshoulder(2) = itP->z;
                break;
          case 2:
                Pelv(0) = itP->x; Pelv(1) = itP->y; Pelv(2) = itP->z;
                break;
          case 3:
                Head(0) = itP->x; Head(1) = itP->y; Head(2) = itP->z;
                break; 
          case 4:
                Neck(0) = itP->x; Neck(1) = itP->y; Neck(2) = itP->z;
                break; 
          case 5:
                Thrx(0) = itP->x; Thrx(1) = itP->y; Thrx(2) = itP->z;
                break; 
          case 6:
                Lhip(0) = itP->x; Lhip(1) = itP->y; Lhip(2) = itP->z;
                break; 
          case 7:
                Lknee(0) = itP->x; Lknee(1) = itP->y; Lknee(2) = itP->z;
                break;
          case 8:
                Lankle(0) = itP->x; Lankle(1) = itP->y; Lankle(2) = itP->z;
                break; 
          case 9:
                Rhip(0) = itP->x; Rhip(1) = itP->y; Rhip(2) = itP->z;
                break; 
          case 10:
                Rknee(0) = itP->x; Rknee(1) = itP->y; Rknee(2) = itP->z;
                break;
          case 11:
                Rankle(0) = itP->x; Rankle(1) = itP->y; Rankle(2) = itP->z;
                break;                              
          }
        }
      }
    }
  }
    void saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
  {
    std::ostringstream oss;
    oss.str("");
    oss << "/home/hts/Desktop/kinect_points/a/aaa/" << std::setfill('0') << std::setw(4);
    const std::string baseName = oss.str();
    const std::string cloudName = baseName + "_cloud.pcd";
    OUT_INFO("saving cloud: " << cloudName);
     // writer是该类的pcl::PCDWriter类型的成员变量
    writer.writeBinary(cloudName, *cloud);
  }
    void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
  {
    if(event.keyUp())
    {
      switch(event.getKeyCode())
      {
      case 27:
      case 'q':
        visualize_stop = true;
        break;
      }
    }
  }
    //叉积
  Vector3f ThreeCross()
  {
    Vector3f a, b,result;
    a=Lshoulder-Pelv;
    b=Rshoulder-Pelv;
    cout<<"左肩向量:\n"<<a<<"   右肩向量:\n"<<b<<endl;
    result = a.cross(b);
    cout<<"躯干法向量:\n"<<result<<endl;
    // Eigen::Vector3d v3(0, 0, 0);
	  // v3.x() = 1;
	  // v3[2] = 1;
	  AngleAxisd angle_axis3(pi *25/ 18, Eigen::Vector3d(1, 0, 0));//1系绕x轴逆时针旋转250得到2系
    // angle_axis3.matrix().cast<float>()
	  Vector3f rotated_result = angle_axis3.matrix().cast<float>()*result;
	  cout << "绕x轴顺时针旋转250°(Rcw):" << endl << angle_axis3.matrix() << endl;
	  cout << "躯干法向量旋转后:" << endl << rotated_result.transpose() << endl;
    return result;
  }
  //转换到世界坐标系
  void  Transformation()
  {
    // Vector3f result = ori;
    // // 旋转矩阵就是3x3的矩阵
    // Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    // 旋转向量(方向与旋转轴[1,0,0]相同,模为旋转的角度250°)
    Eigen::AngleAxisd rotation_vector (pi *25/ 18,Eigen::Vector3d(1,0,0));
    cout.precision(3);//指定输出的精度
    cout<<"Rwc旋转矩阵为：\n"<<rotation_vector.matrix()<<endl;
    // 三维的齐次变换矩阵是4x4的矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    // 根据旋转向量进行旋转（注意旋转向量包含了旋转轴和旋转了的角度）
    T.rotate(rotation_vector);
    cout<<"旋转后的变换矩阵为：\n"<<T.matrix()<<endl;
    T.pretranslate(Eigen::Vector3d(0,0,-1)); // 把第四列的平移向量设置为(1,3,4)
    cout<<"设置平移量后的矩阵为：\n"<<T.matrix()<<endl;
    // result = T*ori;
    Head = T.cast<float>()*Head; Neck = T.cast<float>()*Neck; Thrx = T.cast<float>()*Thrx; Pelv = T.cast<float>()*Pelv; Lhip = T.cast<float>()*Lhip; Lknee = T.cast<float>()*Lknee; Lankle = T.cast<float>()*Lankle; Rhip = T.cast<float>()*Rhip; Rknee = T.cast<float>()*Rknee; Rankle = T.cast<float>()*Rankle; Lshoulder = T.cast<float>()*Lshoulder; Rshoulder = T.cast<float>()*Rshoulder;
  }
  double calculate()
  {
    double sum =0;
    sum = Head(2) + Neck(2) + Thrx(2) + Pelv(2) + Lhip(2) + Lknee(2) + Lankle(2) + Rhip(2) + Rknee(2) + Rankle(2);
    double mean = sum/10;
    double stdvar = sqrt(((Head(2)-mean)*(Head(2)-mean)+(Neck(2)-mean)*(Neck(2)-mean)+(Thrx(2)-mean)*(Thrx(2)-mean)+(Pelv(2)-mean)*(Pelv(2)-mean)+(Lhip(2)-mean)*(Lhip(2)-mean)+(Lknee(2)-mean)*(Lknee(2)-mean)+(Lankle(2)-mean)*(Lankle(2)-mean)+(Rhip(2)-mean)*(Rhip(2)-mean)+(Rknee(2)-mean)*(Rknee(2)-mean)+(Rankle(2)-mean)*(Rankle(2)-mean))/10);
    cout<<"标准差为:"<<stdvar<<endl;
    return stdvar;
  }

};

int main(int argc, char**argv)
{
  std::string color_path = argv[1];
  std::string depth_path = argv[2]; 
  Posepoint posepoint; 
  posepoint.start(color_path, depth_path);
  cout<<"!!";
  posepoint.ThreeCross();
  posepoint.Transformation();
  posepoint.calculate();
}
