// -------------------------------------------
//  @description: 根据彩色图像的像素产生点云代码
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
  double pick_points[6];
public:
  Posepoint():visualize_stop(false)
  {
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    static double my_points[6] = {236.0807342529297, 242.84010314941406,286.8062438964844, 168.4427032470703,351.0585021972656, 209.02308654785156};

    for(int i=0; i<6; i++)
    {
      pick_points[i] = my_points[i];
      // cout<<pick_points[i]<<endl;
    }
  }
  ~Posepoint()
  {
  }
public:
    void start(std::string color_path, std::string depth_path)
  {
    color = cv::imread(color_path);
    depth = cv::imread(depth_path,2);
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
    }
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
    // cout<<fx<<" "<<fy<<" "<<cx<<" "<<cy<<" "<<endl;
    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
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
    // visualizer->addCoordinateSystem(1.0);
    // 通过设置相机参数是用户从默认的角度和方向观察点
    visualizer->initCameraParameters();
    // 设置窗口viewer的背景颜色
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setPosition(0, 0);
    visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->registerKeyboardCallback(&Posepoint::keyboardEvent, *this);
    saveCloud(cloud);
    // visualizer->spinOnce(300);
    while (!visualize_stop) {
        visualizer->spinOnce(100);
    }

    visualizer->close();
  }
    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
  {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();
    FILE *fp = NULL;
    fp = fopen("/home/hts/catkin_ws/test.txt", "a");
    // cout<<depth.rows<<"   "<<depth.cols<<endl;
    // cout<<color.rows<<"   "<<color.cols<<endl;
    //   for(int i = 0; i<3; i++)
    // {
    //   cout<<pick_points[i*2]<<"  "<<pick_points[i*2+1]<<endl;
    // }
    #pragma omp parallel for
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
        bool picked = false;
        for (int i = 0; i < 3; i++)
        {
          // cout<<size_t(pick_points[i*2])<<"  "<<size_t(pick_points[i*2+1])<<endl;
          if(int(pick_points[i*2]) == r && size_t(pick_points[i*2+1]) == c)
          {
            cout<<r<<"   "<<c<<"pick_succ"<<endl;
            picked = true;
          }
        }
        if (picked)
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
          fprintf(fp, "%f,%f,%f,%d,%d,%d,%d\n", itP->z,itP->x,itP->y,itP->b,itP->g,itP->r,itP->a);
        }
        else
        {
            itP->x = itP->y = itP->z = badPoint;
            itP->rgba = 0;
            continue;
        }
        
      }
    }
    fclose(fp);
  }
    void saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
  {
    std::ostringstream oss;
    oss.str("");
    oss << "/home/hts/kinect_pic/" << std::setfill('0') << std::setw(4);
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
};
int main(int argc, char**argv)
{
  std::string color_path = argv[1];
  std::string depth_path = argv[2];
  Posepoint posepoint; 
  posepoint.start(color_path, depth_path);
}