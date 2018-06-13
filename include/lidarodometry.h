#ifndef LIDAR_H
#define LIDAR_H


#include <vector>
#include <deque>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <math.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/registration/ndt.h>     
#include <pcl/registration/icp.h>   //ICP配准类相关头文件
#include <pcl/filters/approximate_voxel_grid.h>   
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件
#include <pcl/console/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
//ROS相关
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "Paramreader.h"

#define arad 0.0174532925199433333333


typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
//typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;   //申明pcl::PointXYZ数据
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;


using namespace std;
using namespace Eigen;


struct FRAME
{
  PointCloud::Ptr cloud;  //点云共享指针
  PointCloud::Ptr filter_cloud;
  PointCloud::Ptr cur_cloud;
  PointCloud::Ptr cur_filter_cloud;
  std::string f_name;   //文件名称
  int frameID;   
  Eigen::Matrix4f  pose;
  FRAME() : cloud (new PointCloud),filter_cloud(new PointCloud), cur_cloud(new PointCloud),cur_filter_cloud(new PointCloud), pose(Eigen::Matrix4f::Identity()) {};
};

struct IMU
{
  int id;
  float X;	
  float Y;	
  float Yaw;	//偏航角
  float Pit;	//俯仰角
  float Rol;	//横滚角
};

struct MATCHPARAM
{
  int TAR;
  int SRC;
  //滤波器参数
  float leafSize;
  //super4pcs参数
  float delta;
  float overlap;
  float loop_overlap;
  float max_translation_distance;
  float max_color;
  float norm_diff;
  float max_angle;
  float thr;
  float loop_thr;
  float min_LCP;
  int n_points;
  int loopn_points;
  int max_time_seconds;
  
  //NDT参数
  float epsilon;
  float stepSize;
  float Resolution;
  int iter_num;
  int match_num;
  
  //ICP参数
  float TransEpsilon;
  float FitnessEpsilon;
  float Distance;
  float icp_num;
  
  //其他参数
  int build_map;
  int start_frame;
  int end_frame;
  
  //存储路径
  string cloud_path;
  string imu_path;
  string out_path;
  string out_trace;
  string out_map;

  bool visualize;
  bool opti;
  bool check_loop_closure;
};



//数字转字符串
string num2str(int i)
{
  stringstream ss;
  ss<<i;
  return ss.str();
}


#endif
