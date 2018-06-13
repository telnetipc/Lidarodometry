#include "lidarodometry.h"


//参数读取
MATCHPARAM MatchParamReader();
//读取KITTI坐标系变换矩阵
Eigen::Matrix4f CaliReader(string imu_path);
//读取IMU参数
void IMUReader(string imu_path, vector<IMU> &imu_data);

//ICP匹配
void ICP_Match(PointCloud::Ptr &source, 
	       PointCloud::Ptr &target, 
	       PointCloud::Ptr &result,
	       Eigen::Matrix4f &pairTransform,
	       MATCHPARAM match_param);
//NDT匹配
void NDT_Match(PointCloud::Ptr &source, 
	       PointCloud::Ptr &target, 
	       PointCloud::Ptr &result,
	       Eigen::Matrix4f &pairTransform,
	       MATCHPARAM match_param);
//滤除地面
void ground_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &laserCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr &outlaserCloud);


void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

//用于校准的局部地图
deque<PointCloud::Ptr> deq_map;
PointCloud::Ptr  local_map(new PointCloud);
//计算算法运行时间
//pcl::console::TicToc time;	
//double sum_time,mean_time;
//存储帧
deque<FRAME> deq_frame;
//用来执行闭环检测的KDtree
bool have_loop = false;
pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
PointCloud::Ptr loop_cloud( new PointCloud );
//存放位姿信息的容器
deque<Eigen::Matrix4f> deq_pose;
//用于匹配的点云及矩阵
PointCloud::Ptr  map_total(new PointCloud);
PointCloud::Ptr  trace_point( new PointCloud );
PointCloud::Ptr  trace_raw( new PointCloud );
PointCloud::Ptr  last_input(new PointCloud);
Eigen::Matrix4f  pairTransform=Eigen::Matrix4f::Identity (), GlobalTransform=Eigen::Matrix4f::Identity ();
Eigen::Matrix4f  lastGlobalTransform = Eigen::Matrix4f::Identity ();
//帧号
size_t frame_i=0;
//标定矩阵
Eigen::Matrix4f CaliT;
//参数读取
MATCHPARAM match_param;

//发布点云消息
ros::Publisher pubLaserCloud;
ros::Publisher pubLaserCloudSurround;
ros::Publisher pubLaserOdometry;

const int systemDelay = 20;
int systemInitCount = 0;
bool systemInited = false;

ros::Time current_time;

int main(int argc, char **argv) {
  
  
  //读取参数
  match_param = MatchParamReader();
  
  
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
    "/pandar_points", 2, laserCloudHandler);

  pubLaserCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);
  
  pubLaserCloudSurround =
        nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
	
  tf::TransformBroadcaster odom_broadcaster;
  pubLaserOdometry =
      nh.advertise<nav_msgs::Odometry>("/integrated_to_init", 5);

  ros::spin();

  return 0;
}


void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount >= systemDelay) {
      systemInited = true;
    }
    return;
  }

  //读入数据
  PointCloud::Ptr  input_cloud(new PointCloud);
  pcl::fromROSMsg(*laserCloudMsg, *input_cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, indices);
  
  
  //***********************************算法开始*************************************//
  PointCloud::Ptr  result(new PointCloud);
  PointCloud::Ptr  result_opti(new PointCloud);
  
  FRAME  current_frame;
  current_frame.frameID = frame_i++;
  cout << frame_i << endl;

  //把高度上错误的点都滤掉
  pcl::PassThrough<pcl::PointXYZI> pass;  
  pass.setInputCloud(input_cloud);  
  pass.setFilterFieldName("z");  
  pass.setFilterLimits(-2.5, 400.0);  
  pass.filter(*input_cloud); 
  pcl::copyPointCloud(*input_cloud, *current_frame.cloud);
  
  
  //开始算法计时
  //time.tic(); 
  
  //去除平面部分
  ground_filter(current_frame.cloud, current_frame.cur_cloud);


  //体素滤波下采样
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setLeafSize (match_param.leafSize, match_param.leafSize, match_param.leafSize);  
  voxel_filter.setInputCloud (current_frame.cur_cloud);
  voxel_filter.filter (*current_frame.cur_filter_cloud);
  voxel_filter.setInputCloud (current_frame.cloud);
  voxel_filter.filter (*current_frame.filter_cloud);

  
  if(current_frame.frameID == 0)	//处理第一帧
  {
    //插入局部地图
    deq_map.push_back(current_frame.filter_cloud);
    deq_pose.push_back(Eigen::Matrix4f::Identity());
    //插入帧队列
    deq_frame.push_back(current_frame);
  }
  else
  {
    ICP_Match(current_frame.cur_filter_cloud, 
	      (*(deq_frame.end()-1)).cur_filter_cloud, 
	      result,
	      pairTransform,
	      match_param);

    //转化到世界坐标系
    GlobalTransform = GlobalTransform * pairTransform;

    
    //局部地图优化
    if(current_frame.frameID % match_param.match_num == 0)
    {
      Eigen::Matrix4f matchPose = lastGlobalTransform.inverse() * GlobalTransform;

      ICP_Match(current_frame.cur_filter_cloud, 
	       (*(deq_frame.end()-match_param.match_num)).cur_filter_cloud, 
	       result,
	       matchPose,
	       match_param);
      
      
      local_map->clear();
      for(size_t t=0;t<deq_map.size();t++)
      {
	Eigen::Matrix4f localPose = Eigen::Matrix4f::Identity ();
	for(size_t j=t+1;j<deq_pose.size();j++)
	{
	  localPose = localPose * deq_pose[j];
	}
	PointCloud::Ptr  tmp_map(new PointCloud);
	Eigen::Matrix4f localPose_inv = localPose.inverse();
	pcl::transformPointCloud(*(deq_map[t]), *tmp_map, localPose_inv);
	*local_map += *(tmp_map);
      }
      
      //NDT匹配优化
      NDT_Match(current_frame.filter_cloud,
	       local_map,
	       result_opti,
	       matchPose,
	       match_param);

      //维护一个5帧的局部地图
      if(deq_map.size()<5)
      {
	deq_map.push_back(current_frame.filter_cloud);
	deq_pose.push_back(matchPose);
      }
      else
      {
	deq_map.pop_front();
	deq_map.push_back(current_frame.filter_cloud);
	deq_pose.pop_front();
	deq_pose.push_back(matchPose);
      }
      
      //更新全局位姿
      GlobalTransform = lastGlobalTransform * matchPose;
       
      //保存当前全局位姿
      lastGlobalTransform = GlobalTransform;
    }
    
    //维护一个5帧的队列，存储历史帧
    if(deq_frame.size() < match_param.match_num)
    {
      deq_frame.push_back(current_frame);
    }
    else
    {
      deq_frame.pop_front();
      deq_frame.push_back(current_frame);
    }
    
  }
    

  PointCloud::Ptr  result_map(new PointCloud);
  pcl::transformPointCloud(*current_frame.cloud, *result_map, GlobalTransform);
  //点云下采样
  voxel_filter.setLeafSize (match_param.leafSize*0.5, match_param.leafSize*0.5, match_param.leafSize*0.5);  
  voxel_filter.setInputCloud (result_map);
  voxel_filter.filter (*result_map);


  //发布当前帧
  sensor_msgs::PointCloud2 laserCloudOutMsg;
  pcl::toROSMsg(*result_map, laserCloudOutMsg);
  laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
  laserCloudOutMsg.header.frame_id = "/camera_init";
  pubLaserCloud.publish(laserCloudOutMsg);

  //发布全局地图 
  if(current_frame.frameID % match_param.build_map == 0)
  {
    *map_total += *result_map;
    
    sensor_msgs::PointCloud2 laserCloudSurround3;
    pcl::toROSMsg(*map_total, laserCloudSurround3);
    laserCloudSurround3.header.stamp = laserCloudMsg->header.stamp;
    laserCloudSurround3.header.frame_id = "/camera_init";
    pubLaserCloudSurround.publish(laserCloudSurround3);
  }
  
  
  Eigen::Matrix3f t_R;
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      t_R(i,j)=GlobalTransform(i,j);
    }
  }
  Eigen::Quaternionf geoQ(t_R);
  
  nav_msgs::Odometry odom;
  odom.header.stamp = laserCloudMsg->header.stamp;
  odom.header.frame_id = "/camera_init";
  
  odom.pose.pose.position.x = GlobalTransform(0,3);
  odom.pose.pose.position.y = GlobalTransform(1,3);
  odom.pose.pose.position.z = GlobalTransform(2,3);
  odom.pose.pose.orientation.x = geoQ.x();
  odom.pose.pose.orientation.y = geoQ.y();
  odom.pose.pose.orientation.z = geoQ.z();
  odom.pose.pose.orientation.w = geoQ.w();

  pubLaserOdometry.publish(odom);
  
  
/*  //计算时间
  float timecost =time.toc()/1000;
  sum_time+=timecost;
  mean_time = sum_time / current_frame.frameID;
  cout<< timecost <<"s  " << sum_time <<"s  "<< mean_time <<"s"<<endl;*/


//   //写入文件   
//   Eigen::Matrix4f outTransform = CaliT * GlobalTransform * CaliT.inverse();
//   //outTransform(1,3)=0;
//   outfile << setprecision (6) << scientific 
// 	  << outTransform(0,0) << " " << outTransform(0,1) << " "  << outTransform(0,2) << " " << outTransform(0,3) << " "
// 	  << outTransform(1,0) << " " << outTransform(1,1) << " "  << outTransform(1,2) << " " << outTransform(1,3) << " "
// 	  << outTransform(2,0) << " " << outTransform(2,1) << " "  << outTransform(2,2) << " " << outTransform(2,3) <<  endl;

}  



void NDT_Match(PointCloud::Ptr &source, 
	       PointCloud::Ptr &target, 
	       PointCloud::Ptr &result,
	       Eigen::Matrix4f &pairTransform,
	       MATCHPARAM match_param)
{
  //NDT匹配
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  ndt.setTransformationEpsilon (match_param.epsilon);   
  ndt.setStepSize (match_param.stepSize);   
  ndt.setResolution (match_param.Resolution);  
  ndt.setMaximumIterations (match_param.iter_num);
  ndt.setInputSource (source); 
  ndt.setInputTarget (target);               
  
  ndt.align (*result, pairTransform);
  
  pairTransform=ndt.getFinalTransformation();
}

void ICP_Match(PointCloud::Ptr &source, 
	       PointCloud::Ptr &target, 
	       PointCloud::Ptr &result,
	       Eigen::Matrix4f &pairTransform,
	       MATCHPARAM match_param)
{
  //ICP匹配
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(match_param.Distance);
  icp.setTransformationEpsilon(match_param.TransEpsilon);
  icp.setEuclideanFitnessEpsilon(match_param.FitnessEpsilon);
  icp.setMaximumIterations(match_param.icp_num);	
  
  icp.setInputSource(source);                
  icp.setInputTarget(target);              
  
  icp.align (*result, pairTransform);
  
  pairTransform=icp.getFinalTransformation();
}


MATCHPARAM MatchParamReader()
{
  //参数读取
  static ParameterReader pd;
  MATCHPARAM match_param;
  
  match_param.TAR = atoi( pd.getData( "TAR" ).c_str() );
  match_param.SRC = atoi( pd.getData( "SRC" ).c_str() );
  //滤波器参数
  match_param.leafSize = atof( pd.getData( "leafSize" ).c_str() );
  //super4pcs参数
  match_param.delta = atof( pd.getData( "delta" ).c_str() );
  match_param.overlap = atof( pd.getData( "overlap" ).c_str() );
  match_param.loop_overlap = atof( pd.getData( "loop_overlap" ).c_str() );
  match_param.max_translation_distance = atof( pd.getData( "max_translation_distance" ).c_str() );
  match_param.max_color = atof( pd.getData( "max_color" ).c_str() );
  match_param.norm_diff = atof( pd.getData( "norm_diff" ).c_str() );
  match_param.max_angle = atof( pd.getData( "max_angle" ).c_str() );
  match_param.thr = atof( pd.getData( "thr" ).c_str() );
  match_param.loop_thr = atof( pd.getData( "loop_thr" ).c_str() );
  match_param.min_LCP = atof( pd.getData( "min_LCP" ).c_str() );
  match_param.n_points = atoi( pd.getData( "n_points" ).c_str() );
  match_param.max_time_seconds = atoi( pd.getData( "max_time_seconds" ).c_str() );
  
  //NDT参数
  match_param.epsilon = atof( pd.getData( "epsilon" ).c_str() );
  match_param.stepSize = atof( pd.getData( "stepSize" ).c_str() );
  match_param.Resolution = atof( pd.getData( "Resolution" ).c_str() );
  match_param.iter_num = atoi( pd.getData( "iter_num" ).c_str() );
  
  
  //ICP参数
  match_param.TransEpsilon = atof( pd.getData( "TransEpsilon" ).c_str() );
  match_param.FitnessEpsilon = atof( pd.getData( "FitnessEpsilon" ).c_str() );
  match_param.Distance = atof( pd.getData( "Distance" ).c_str() );
  match_param.icp_num = atof( pd.getData( "icp_num" ).c_str() );
  
  //其他参数
  match_param.match_num = atoi( pd.getData( "match_num" ).c_str() );
  match_param.build_map = atoi( pd.getData( "build_map" ).c_str() );
  match_param.start_frame = atoi( pd.getData( "start_frame" ).c_str() );
  match_param.end_frame = atoi( pd.getData( "end_frame" ).c_str() );
  //存储路径
  match_param.cloud_path = pd.getData( "cloud_path" ).c_str();
  match_param.imu_path = pd.getData( "imu_path" ).c_str();
  match_param.out_path = pd.getData( "out_path" ).c_str();
  match_param.out_trace = pd.getData( "out_trace" ).c_str();
  match_param.out_map = pd.getData( "out_map" ).c_str();
  // 是否显示点云
  match_param.visualize = pd.getData("visualize_pointcloud")==string("yes");
  match_param.opti = pd.getData("opti")==string("yes");
  match_param.check_loop_closure = pd.getData("check_loop_closure")==string("yes");
  
  cout <<"start_frame: "<< match_param.start_frame << endl;
  cout <<"end_frame: "<< match_param.end_frame << endl;
  
  
  return match_param; 
}


Eigen::Matrix4f CaliReader(string imu_path)
{
  //读取IMU数据
  ifstream imu_file1(imu_path);
  
  vector<string> s_imu_data;
  vector<float> f_imu_data;
  
  string temp;
  while(getline(imu_file1,temp))
    s_imu_data.push_back(temp);
  
  //将string数据转换成float数据
  for(auto it = s_imu_data.begin(); it != s_imu_data.end(); it++)
  {
    //cout << *it << ' ';
    istringstream wrt_record(*it);
    float f_temp;
    while(wrt_record >> f_temp)
      f_imu_data.push_back(f_temp);
  }
  vector<string> ().swap(s_imu_data);	//释放容器空间
    
  
  Eigen::Matrix4f CaliTr = Eigen::Matrix4f::Identity ();
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      CaliTr(i,j)=f_imu_data[4*i+j];
    }
  }
  vector<float> ().swap(f_imu_data);	//释放容器空间
  
  
  Eigen::Matrix4f CaliT = CaliTr;
  
  return CaliT;
}


void IMUReader(string imu_path, vector<IMU> &imu_data)
{
  ifstream imu_file1(imu_path);
  
  vector<string> s_imu_data;
  vector<float> f_imu_data;
  
  string temp;
  while(getline(imu_file1,temp))
    s_imu_data.push_back(temp);
  
  //将string数据转换成float数据
  for(auto it = s_imu_data.begin(); it != s_imu_data.end(); it++)
  {
    //cout << *it << ' ';
    istringstream wrt_record(*it);
    float f_temp;
    while(wrt_record >> f_temp)
      f_imu_data.push_back(f_temp);
  }
  vector<string> ().swap(s_imu_data);	//释放容器空间
  

  for(auto it = f_imu_data.begin(); it != f_imu_data.end(); it = it + 6)
  {
    IMU imu_temp;
    imu_temp.id = *it;
    imu_temp.Rol = *(it+1);
    imu_temp.Pit = *(it+2);
    imu_temp.Yaw = *(it+3);
    imu_temp.X = *(it+4);
    imu_temp.Y = *(it+5);
    
    imu_data.push_back(imu_temp);
  }
  vector<float> ().swap(f_imu_data);	//释放容器空间
}



void ground_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &laserCloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr &outlaserCloud)
{
  vector<vector<vector<pcl::PointXYZI>>> s(720, vector<vector<pcl::PointXYZI>>(250));
  
  for (int i = 0; i < laserCloudIn->size(); i++) {
    pcl::PointXYZI point;
    point.x = laserCloudIn->points[i].x;
    point.y = laserCloudIn->points[i].y;
    point.z = laserCloudIn->points[i].z;
    
    
    float ori = atan2(point.y, point.x)* 180 / M_PI;
    if(ori<0)
      ori=ori+360;
    
    int m=ori / 0.5;
    if(m>=720)
      m=719;
    
    float dis=sqrt(point.x*point.x+point.y*point.y);
    if(dis>70)
      continue;
    
    int n=(dis-0)/0.28;
    if(n>=250)
      n=249;
    
    (s[m][n]).push_back(point);

  }
  
  for(int i=0;i<720;i++){
    for(int j=0;j<250;j++){
      float max=-999,min=999;
      for(int k=0;k<s[i][j].size();k++){
	if(s[i][j].at(k).z>max){
	  max=s[i][j].at(k).z;
	}
	if(s[i][j].at(k).z<min){
	  min=s[i][j].at(k).z;
	}
      }
      
      if(max==-999||min==999)
	continue;
      
      float delta_h=max-min;
      
      if(delta_h>0.1){
	for(int l=0;l<s[i][j].size();l++){
	  outlaserCloud->push_back(s[i][j].at(l));
	}
      }
    }
  }
  
}
