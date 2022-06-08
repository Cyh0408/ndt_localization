#include "ndt.h"

NdtLocalizer::NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh):nh_(nh), private_nh_(private_nh), tf2_listener_(tf2_buffer_){

  key_value_stdmap_["state"] = "Initializing";
  init_params();

  // Publishers
  sensor_aligned_pose_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_aligned", 10);
  ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);
  exe_time_pub_ = nh_.advertise<std_msgs::Float32>("exe_time_ms", 10);
  transform_probability_pub_ = nh_.advertise<std_msgs::Float32>("transform_probability", 10);
  iteration_num_pub_ = nh_.advertise<std_msgs::Float32>("iteration_num", 10);
  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  path_pub = nh_.advertise<nav_msgs::Path>("trajectory",1, true);


  // Subscribers
  // 从rviz中获取初始位姿信息
  // 从maploader中获取pcd点云话题
  // 从voxel_grid_filter获取降采样后的点云
  initial_pose_sub_ = nh_.subscribe("initialpose", 100, &NdtLocalizer::callback_init_pose, this);
  map_points_sub_ = nh_.subscribe("points_map", 1, &NdtLocalizer::callback_pointsmap, this);
  sensor_points_sub_ = nh_.subscribe("filtered_points", 1, &NdtLocalizer::callback_pointcloud, this);
  gnss_pose_sub_ = nh_.subscribe("gps_pose", 100, &NdtLocalizer::gnss_callback, this);
  

  diagnostic_thread_ = std::thread(&NdtLocalizer::timer_diagnostic, this);
  diagnostic_thread_.detach();
}

NdtLocalizer::~NdtLocalizer() {}

void NdtLocalizer::timer_diagnostic()
{
  ros::Rate rate(100);
  while (ros::ok()) {
    diagnostic_msgs::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "ndt_scan_matcher";
    diag_status_msg.hardware_id = "";

    for (const auto & key_value : key_value_stdmap_) {
      diagnostic_msgs::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing") {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_status_msg.message += "Initializing State. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1) {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5) {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }

    diagnostic_msgs::DiagnosticArray diag_msg;
    diag_msg.header.stamp = ros::Time::now();
    diag_msg.status.push_back(diag_status_msg);

    diagnostics_pub_.publish(diag_msg);

    rate.sleep();
  }
}

// rviz初始化
// 用于对初始位姿变换到map坐标系下，并用initial_pose_cov_msg_表示
void NdtLocalizer::callback_init_pose(
  // 从rviz中获取到初始位姿后调用，输入为初始位姿信息
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & initial_pose_msg_ptr)
{
  // 如果输入的位姿是"map"坐标系下的，将其赋值给initial_pose_cov_msg_
  if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
    initial_pose_cov_msg_ = *initial_pose_msg_ptr;
 
  } else {
    // get TF from pose_frame to map_frame
    // 如果输入的位姿不是"map"坐标系下的，将其转到"map"坐标系下后再赋值给initial_pose_cov_msg_
    // 获取位姿坐标系与"map"坐标系之间的tf变换，并存到TF_pose_to_map_ptr
    geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
    get_transform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    // transform pose_frame to map_frame
    // 利用TF_pose_to_map_ptr将输入位姿由位姿坐标转到"map"坐标系，新位姿有*mapTF_initial_pose_msgs_ptr表示
    geometry_msgs::PoseWithCovarianceStamped::Ptr mapTF_initial_pose_msg_ptr(
      new geometry_msgs::PoseWithCovarianceStamped);
    tf2::doTransform(*initial_pose_msg_ptr, *mapTF_initial_pose_msg_ptr, *TF_pose_to_map_ptr);
    // mapTF_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
    // 将转到"map"坐标系后的位姿赋值给initial_pose_cov_msg_
    initial_pose_cov_msg_ = *mapTF_initial_pose_msg_ptr;

  }
  // if click the initpose again, re init！
  //initialpose = false;
  init_pose = false;
}

// gnss初始化
void NdtLocalizer::gnss_callback(const geometry_msgs::PoseStamped::ConstPtr & gps_msg_ptr)
{
  if(!init_pose) {
  if(gps_msg_ptr->header.frame_id == map_frame_){
    initial_pose_msg_ = *gps_msg_ptr;
    //std::cout << "allways" << std::endl;
  } else {
    geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
    get_transform(map_frame_, gps_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    geometry_msgs::PoseStamped::Ptr mapTF_initial_pose_msg_ptr(
      new geometry_msgs::PoseStamped);
    tf2::doTransform(*gps_msg_ptr, *mapTF_initial_pose_msg_ptr, *TF_pose_to_map_ptr);
    initial_pose_msg_ = *mapTF_initial_pose_msg_ptr;
  }
  init_pose = false;
  //std::cout << " 1:  "<< GNSS_SUBSCRIBER <<std::endl;
  // GNSS_SUBSCRIBER = true;
  //  std::cout << " 2:  "<< GNSS_SUBSCRIBER <<std::endl;
  }
}

// 用于将pcd文件点云设置为ndt的目标点云，设置ndt各参数
void NdtLocalizer::callback_pointsmap(
  const sensor_msgs::PointCloud2::ConstPtr & map_points_msg_ptr) // 从maploader节点获取到pcd文件的点云话题后调用，输入为pcd文件内的点云
{
  // ndt_在ndt.h中定义: pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_
  const auto trans_epsilon = ndt_.getTransformationEpsilon(); // 最小搜索变化量
  const auto step_size = ndt_.getStepSize(); // 搜索步长
  const auto resolution = ndt_.getResolution(); // 目标点云的ND体素，单位为m
  const auto max_iterations = ndt_.getMaximumIterations(); // 使用牛顿迭代法的迭代次数

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_new;
  // 更新ndt_
  ndt_new.setTransformationEpsilon(trans_epsilon);
  ndt_new.setStepSize(step_size);
  ndt_new.setResolution(resolution);
  ndt_new.setMaximumIterations(max_iterations);

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // 将输入的pcd文件内的ROS类型点云转到*map_points_ptr
  pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
  // 将map_points_ptr所指点云作为ndt的目标点云
  ndt_new.setInputTarget(map_points_ptr);
  // create Thread
  // detach
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt_new.align(*output_cloud, Eigen::Matrix4f::Identity());

  // swap
  ndt_map_mtx_.lock();
  ndt_ = ndt_new;
  ndt_map_mtx_.unlock();
}

void NdtLocalizer::callback_pointcloud(
  // 从voxel_grid_filter节点获取到降采样后的velodyne点云信息后调用，输入为降采样后的points_raw
  const sensor_msgs::PointCloud2::ConstPtr & sensor_points_sensorTF_msg_ptr)
{
  const auto exe_start_time = std::chrono::system_clock::now();
  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);
  // 接收到的points_raw的frame_id
  // 接收到的points_raw的时间戳
  const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
  const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  // 将接收的到ROS类型的点云转到*sensor_points_sensorTF_ptr
  pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
  // get TF base to sensor
  // 获取velodyne到base的tf变换，并存到TF_base_to_sensor_ptr
  geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr(new geometry_msgs::TransformStamped);
  get_transform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);

  const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
  // 从velodyne到base的转换矩阵
  const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_baselinkTF_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(
    // 将points_raw点云通过base_to_sensor_matrix转换到base坐标系，结果保存到*sensor_points_baselinkTF_ptr
    *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);
  
  // set input point cloud
  // 将base坐标系下的points_raw设置为ndt的输入源; (the ndt target is the modified pointcloud in pcd map)
  ndt_.setInputSource(sensor_points_baselinkTF_ptr);

  if (ndt_.getInputTarget() == nullptr) {
    ROS_WARN_STREAM_THROTTLE(1, "No MAP!");
    return;
  }
  // align
  Eigen::Matrix4f initial_pose_matrix;
  if (!init_pose){
    Eigen::Affine3d initial_pose_affine;
    // 将rviz中获取到的ROS类型的初始位姿转为矩阵形式
    // if (!initialpose) {
    //   tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
    // } else {
    tf2::fromMsg(initial_pose_msg_.pose, initial_pose_affine);
    //tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
    // }
    initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
    // for the first time, we don't know the pre_trans, so just use the init_trans, 
    // which means, the delta trans for the second time is 0
    pre_trans = initial_pose_matrix;
    init_pose = true;
  }else
  {
    // use predicted pose as init guess (currently we only impl linear model)
    // 将上一帧求得的位姿作为初始位姿，利用线性模型做当前帧位姿的估计
    initial_pose_matrix = pre_trans * delta_trans;
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  const auto align_start_time = std::chrono::system_clock::now();
  key_value_stdmap_["state"] = "Aligning";
  ndt_.align(*output_cloud, initial_pose_matrix);
  key_value_stdmap_["state"] = "Sleeping";
  const auto align_end_time = std::chrono::system_clock::now();
  // ndt配准总时长
  const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() /1000.0;
  // 将ndt配准的最终结果转为矩阵形式，存到result_pose_matrix
  const Eigen::Matrix4f result_pose_matrix = ndt_.getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  // 将ndt最终结果转为ROS类型并发布
  const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

  const float transform_probability = ndt_.getTransformationProbability();
  const int iteration_num = ndt_.getFinalNumIteration();
 
  // 收敛判别
  bool is_converged = true;
  static size_t skipping_publish_num = 0;
  if (
    iteration_num >= ndt_.getMaximumIterations() + 2 ||
    transform_probability < converged_param_transform_probability_) {
    is_converged = false;
    ++skipping_publish_num;
    std::cout << "Not Converged" << std::endl;
  } else {
    skipping_publish_num = 0;
  }
  // calculate the delta tf from pre_trans to current_trans
  // 求用于下一帧线性计算需要的delta_trans=pre_trans^-1 * result_pose_matrix
  delta_trans = pre_trans.inverse() * result_pose_matrix;

  Eigen::Vector3f delta_translation = delta_trans.block<3, 1>(0, 3);
  std::cout<<"delta x: "<<delta_translation(0) << " y: "<<delta_translation(1)<<
             " z: "<<delta_translation(2)<<std::endl;

  Eigen::Matrix3f delta_rotation_matrix = delta_trans.block<3, 3>(0, 0);
  Eigen::Vector3f delta_euler = delta_rotation_matrix.eulerAngles(2,1,0);
  std::cout<<"delta yaw: "<<delta_euler(0) << " pitch: "<<delta_euler(1)<<
             " roll: "<<delta_euler(2)<<std::endl;
  // 更新pre_trans
  pre_trans = result_pose_matrix;
  
  // publish
  geometry_msgs::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time; // points_raw的接收时间戳
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg; // the result pose
  
  // 发布ndt_pose轨迹，显示在rviz
  path.header.stamp = sensor_ros_time;
  path.header.frame_id = map_frame_;
  path.poses.push_back(result_pose_stamped_msg);
  
  // 如果收敛，发布带有frame_id和时间戳信息的配准结果
  if (is_converged) {
    ndt_pose_pub_.publish(result_pose_stamped_msg);
    path_pub.publish(path);
  }

  // publish tf(map frame to base frame)
  publish_tf(map_frame_, base_frame_, result_pose_stamped_msg);

  // publish aligned point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(
    // 将base坐标系下的points_raw通过result_pose_matrix转换到map坐标系下，保存到*sensor_points_mapTF_ptr
    *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
  sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
  // 将map坐标系下的points_raw转为ROS类型的点云，存到sensor_points_mapTF_msg
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = map_frame_;
  // 发布带有时间戳和frame_id的map坐标系下的points_raw
  sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);


  std_msgs::Float32 exe_time_msg;
  exe_time_msg.data = exe_time;
  exe_time_pub_.publish(exe_time_msg);

  std_msgs::Float32 transform_probability_msg;
  transform_probability_msg.data = transform_probability;
  transform_probability_pub_.publish(transform_probability_msg);

  std_msgs::Float32 iteration_num_msg;
  iteration_num_msg.data = iteration_num;
  iteration_num_pub_.publish(iteration_num_msg);

  key_value_stdmap_["seq"] = std::to_string(sensor_points_sensorTF_msg_ptr->header.seq);
  key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
  key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);

  std::cout << "------------------------------------------------" << std::endl;
  std::cout << "align_time: " << align_time << "ms" << std::endl;
  std::cout << "exe_time: " << exe_time << "ms" << std::endl;
  std::cout << "trans_prob: " << transform_probability << std::endl;
  std::cout << "iter_num: " << iteration_num << std::endl;
  std::cout << "skipping_publish_num: " << skipping_publish_num << std::endl;
}

void NdtLocalizer::init_params(){

  private_nh_.getParam("base_frame", base_frame_);
  ROS_INFO("base_frame_id: %s", base_frame_.c_str());

  double trans_epsilon = ndt_.getTransformationEpsilon();
  double step_size = ndt_.getStepSize();
  double resolution = ndt_.getResolution();
  int max_iterations = ndt_.getMaximumIterations();

  private_nh_.getParam("trans_epsilon", trans_epsilon);
  private_nh_.getParam("step_size", step_size);
  private_nh_.getParam("resolution", resolution);
  private_nh_.getParam("max_iterations", max_iterations);

  map_frame_ = "map";

  ndt_.setTransformationEpsilon(trans_epsilon);
  ndt_.setStepSize(step_size);
  ndt_.setResolution(resolution);
  ndt_.setMaximumIterations(max_iterations);

  ROS_INFO(
    "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon,
    step_size, resolution, max_iterations);

  private_nh_.getParam(
    "converged_param_transform_probability", converged_param_transform_probability_);
}


bool NdtLocalizer::get_transform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr, const ros::Time & time_stamp)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, time_stamp);
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool NdtLocalizer::get_transform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

void NdtLocalizer::publish_tf(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::PoseStamped & pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_localizer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    NdtLocalizer ndt_localizer(nh, private_nh);

    ros::spin();

    return 0;
}