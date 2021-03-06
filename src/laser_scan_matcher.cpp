/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*  This package uses Canonical Scan Matcher [1], written by
 *  Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */

#include <laser_scan_matcher/laser_scan_matcher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/assign.hpp>
//时间同步
/*#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
*/
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/time_synchronizer.h>
#include <vector>
#include <laserscan_pro/LaserScanPro.h>


namespace scan_tools
{

LaserScanMatcher::LaserScanMatcher(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  initialized_(false),
  received_imu_(false),
  received_odom_(false),
  state_imu_(false),
  flag(0),
  state_last_imu_(true),
  state_odom_(false),
  state_last_odom_(true),
  received_vel_(false)
{
  ROS_INFO("Starting LaserScanMatcher");

  // **** init parameters

  initParams();

  //时间同步测试（use_imu use_odom 写为false）
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,sensor_msgs::Imu> slamSyncPolicy;
  // message_filters::Subscriber<sensor_msgs::LaserScan>* laser_sub_ ;             // topic1 输入
  // message_filters::Subscriber<sensor_msgs::Imu>* imu_sub_;   // topic2 输入
  // message_filters::Synchronizer<slamSyncPolicy>* sync_;

  // laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "/scan", 1);
  // imu_sub_  = new message_filters::Subscriber<sensor_msgs::Imu>(nh_, "/imu", 1);
   
  // sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(10), *laser_sub_, *imu_sub_);
  // sync_->registerCallback(boost::bind(&LaserScanMatcher::CombineCallback,this, _1, _2));
  // message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_(nh_, "/scan", 1);            // topic1 输入
  // message_filters::Subscriber<sensor_msgs::Imu> imu_sub_(nh_, "/imu", 1);   // topic2 输入

  // message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::Image> sync(laser_sub_, imu_sub_, 10);       // 同步
  // // sync.registerCallback(boost::bind(&callback, _1, _2));                   // 回调
  // scan_subscriber_ = nh_.subscribe(
  //     "scan", 1, &LaserScanMatcher::scanCallback, this);


  // **** state variables

  f2b_.setIdentity();
  f2b_kf_.setIdentity();
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

  // Initialize output_ vectors as Null for error-checking
  output_.cov_x_m = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;

  // **** publishers

  if (publish_pose_)
  {
    pose_publisher_  = nh_.advertise<geometry_msgs::Pose2D>(
      "pose2D", 5);
  }

  if (publish_laserscanpro_)
  {
    laserscanpro_publisher_ = nh_.advertise<laserscan_pro::LaserScanPro>(
      "scanpro", 5);
  }

  if (publish_pose_stamped_)
  {
    pose_stamped_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "pose_stamped", 5);
  }

  if (publish_pose_with_covariance_)
  {
    pose_with_covariance_publisher_  = nh_.advertise<geometry_msgs::PoseWithCovariance>(
      "pose_with_covariance", 5);
  }

  if (publish_pose_with_covariance_stamped_)
  {
    pose_with_covariance_stamped_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "pose_with_covariance_stamped", 5);
  }

  // *** subscribers

  if (use_cloud_input_)
  {
    cloud_subscriber_ = nh_.subscribe(
      "cloud", 1, &LaserScanMatcher::cloudCallback, this);
  }
  else
  {
    scan_subscriber_ = nh_.subscribe(
      "scan", 1, &LaserScanMatcher::scanCallback, this);
  }

  if (use_imu_)
  {
    imu_subscriber_ = nh_.subscribe(
      "imu", 1, &LaserScanMatcher::imuCallback, this);
  }
  if (use_odom_)
  {
    odom_subscriber_ = nh_.subscribe(
      "odomgen", 1, &LaserScanMatcher::odomCallback, this);
  }
  if (use_vel_)
  {
    if (stamped_vel_)
      vel_subscriber_ = nh_.subscribe(
        "vel", 1, &LaserScanMatcher::velStmpCallback, this);
    else
      vel_subscriber_ = nh_.subscribe(
        "vel", 1, &LaserScanMatcher::velCallback, this);
  }
}

LaserScanMatcher::~LaserScanMatcher()
{
  ROS_INFO("Destroying LaserScanMatcher");
}


void LaserScanMatcher::initParams()
{
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "world";

  // **** input type - laser scan, or point clouds?
  // if false, will subscribe to LaserScan msgs on /scan.
  // if true, will subscribe to PointCloud2 msgs on /cloud

  if (!nh_private_.getParam ("use_cloud_input", use_cloud_input_))
    use_cloud_input_= false;

  if (use_cloud_input_)
  {
    if (!nh_private_.getParam ("cloud_range_min", cloud_range_min_))
      cloud_range_min_ = 0.1;
    if (!nh_private_.getParam ("cloud_range_max", cloud_range_max_))
      cloud_range_max_ = 50.0;
    if (!nh_private_.getParam ("cloud_res", cloud_res_))
      cloud_res_ = 0.05;

    input_.min_reading = cloud_range_min_;
    input_.max_reading = cloud_range_max_;
  }

  // **** keyframe params: when to generate the keyframe scan
  // if either is set to 0, reduces to frame-to-frame matching

  if (!nh_private_.getParam ("kf_dist_linear", kf_dist_linear_))
    kf_dist_linear_ = 0.10;
  if (!nh_private_.getParam ("kf_dist_angular", kf_dist_angular_))
    kf_dist_angular_ = 10.0 * (M_PI / 180.0);

  kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;

  // **** What predictions are available to speed up the ICP?
  // 1) imu - [theta] from imu yaw angle - /imu topic
  // 2) odom - [x, y, theta] from wheel odometry - /odom topic
  // 3) vel - [x, y, theta] from velocity predictor - see alpha-beta predictors - /vel topic
  // If more than one is enabled, priority is imu > odom > vel

  if (!nh_private_.getParam ("use_imu", use_imu_))
    use_imu_ = true;
  if (!nh_private_.getParam ("use_odom", use_odom_))
    use_odom_ = true;
  if (!nh_private_.getParam ("use_vel", use_vel_))
    use_vel_ = false;

  // **** Are velocity input messages stamped?
  // if false, will subscribe to Twist msgs on /vel
  // if true, will subscribe to TwistStamped msgs on /vel
  if (!nh_private_.getParam ("stamped_vel", stamped_vel_))
    stamped_vel_ = false;

  // **** How to publish the output?
  // tf (fixed_frame->base_frame),
  // pose message (pose of base frame in the fixed frame)

  if(!nh_private_.getParam ("publesh_laserscanpro",publish_laserscanpro_))
    publish_laserscanpro_ = true;
  if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = true;
  if (!nh_private_.getParam ("publish_pose", publish_pose_))
    publish_pose_ = false;
  if (!nh_private_.getParam ("publish_pose_stamped", publish_pose_stamped_))
    publish_pose_stamped_ = false;
  if (!nh_private_.getParam ("publish_pose_with_covariance", publish_pose_with_covariance_))
    publish_pose_with_covariance_ = false;
  if (!nh_private_.getParam ("publish_pose_with_covariance_stamped", publish_pose_with_covariance_stamped_))
    publish_pose_with_covariance_stamped_ = false;

  if (!nh_private_.getParam("position_covariance", position_covariance_))
  {
    position_covariance_.resize(3);
    std::fill(position_covariance_.begin(), position_covariance_.end(), 1e-9);
  }

  if (!nh_private_.getParam("orientation_covariance", orientation_covariance_))
  {
    orientation_covariance_.resize(3);
    std::fill(orientation_covariance_.begin(), orientation_covariance_.end(), 1e-9);
  }
  // **** CSM parameters - comments copied from algos.h (by Andrea Censi)

  // Maximum angular displacement between scans
  if (!nh_private_.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  if (!nh_private_.getParam ("max_linear_correction", input_.max_linear_correction))
    input_.max_linear_correction = 0.50;

  // Maximum ICP cycle iterations
  if (!nh_private_.getParam ("max_iterations", input_.max_iterations))
    input_.max_iterations = 10;

  // A threshold for stopping (m)
  if (!nh_private_.getParam ("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = 0.000001;

  // A threshold for stopping (rad)
  if (!nh_private_.getParam ("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = 0.000001;

  // Maximum distance for a correspondence to be valid
  if (!nh_private_.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
    input_.max_correspondence_dist = 0.3;

  // Noise in the scan (m)
  if (!nh_private_.getParam ("sigma", input_.sigma))
    input_.sigma = 0.010;

  // Use smart tricks for finding correspondences.
  if (!nh_private_.getParam ("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!nh_private_.getParam ("restart", input_.restart))
    input_.restart = 0;

  // Restart: Threshold for restarting
  if (!nh_private_.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!nh_private_.getParam ("restart_dt", input_.restart_dt))
    input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!nh_private_.getParam ("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!nh_private_.getParam ("clustering_threshold", input_.clustering_threshold))
    input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  if (!nh_private_.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = 20;

  // If 0, it's vanilla ICP
  if (!nh_private_.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  if (!nh_private_.getParam ("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = 0;

  // Discard correspondences based on the angles - threshold angle, in degrees
  if (!nh_private_.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = 20.0;

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  if (!nh_private_.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = 0.90;

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  if (!nh_private_.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = 0.7;

  if (!nh_private_.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = 2.0;

  // If you already have a guess of the solution, you can compute the polar angle
  // of the points of one scan in the new position. If the polar angle is not a monotone
  // function of the readings index, it means that the surface is not visible in the
  // next position. If it is not visible, then we don't use it for matching.
  if (!nh_private_.getParam ("do_visibility_test", input_.do_visibility_test))
    input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!nh_private_.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  if (!nh_private_.getParam ("do_compute_covariance", input_.do_compute_covariance))
    input_.do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  if (!nh_private_.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
    input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  if (!nh_private_.getParam ("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  if (!nh_private_.getParam ("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = 0;
}

void LaserScanMatcher::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  boost::mutex::scoped_lock(mutex_);
  // std::cerr<<"imu start here"<<std::endl;
  latest_imu_msg_ = *imu_msg;
  if(state_imu_==state_last_imu_){
    imu_data.push_back(latest_imu_msg_);
    // std::cerr<<"state_imu_==state_last_imu_"<<std::endl;
    
  }else{
    // flag =1;
    // imu_data.push_back(latest_imu_msg_);
    // imu_last_data = imu_data;
    state_last_imu_ = state_imu_;
    imu_data.clear();
    imu_data.push_back(latest_imu_msg_);
  }
  if (!received_imu_)
  {
    last_used_imu_msg_ = *imu_msg;
    received_imu_ = true;
  }
}

void LaserScanMatcher::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  // boost::mutex::scoped_lock(mutex_);
  latest_odom_msg_ = *odom_msg;

  if(state_odom_==state_last_odom_){
    odom_data.push_back(latest_odom_msg_);
  }else{
    state_last_odom_ = state_odom_;
    odom_data.clear();
    // odom_data.~vector();
    
    odom_data.push_back(latest_odom_msg_);
  }

  if (!received_odom_)
  {
    last_used_odom_msg_ = *odom_msg;
    received_odom_ = true;
  }
}

void LaserScanMatcher::velCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
  boost::mutex::scoped_lock(mutex_);
  latest_vel_msg_ = *twist_msg;

  received_vel_ = true;
}

void LaserScanMatcher::velStmpCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg)
{
  boost::mutex::scoped_lock(mutex_);
  latest_vel_msg_ = twist_msg->twist;

  received_vel_ = true;
}

void LaserScanMatcher::cloudCallback (const PointCloudT::ConstPtr& cloud)
{
  // **** if first scan, cache the tf from base to the scanner

  std_msgs::Header cloud_header = pcl_conversions::fromPCL(cloud->header);

  if (!initialized_)
  {
    // cache the static tf from base to laser
    if (!getBaseToLaserTf(cloud_header.frame_id))
    {
      ROS_WARN("Skipping scan");
      return;
    }

    PointCloudToLDP(cloud, prev_ldp_scan_);
    last_icp_time_ = cloud_header.stamp;
    initialized_ = true;
  }

  LDP curr_ldp_scan;
  PointCloudToLDP(cloud, curr_ldp_scan);
  processScan(curr_ldp_scan, cloud_header.stamp);
}
//之前的scanCalback
// void LaserScanMatcher::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
// {
//   state_imu_ = !state_imu_;
//       // std::cerr<<"state_imu_ != state_last_imu_"<<std::endl;
//   //     if(flag){
//   //   std::cerr<<"imu_last_data sample start time "<<imu_last_data[0].header.stamp<<std::endl;
//   //   std::cerr<<"imu_last_data sample start+1 time "<<imu_last_data[1].header.stamp<<std::endl;
//   //   std::cerr<<"imu_last_data sample stop time "<<imu_last_data.back().header.stamp<<std::endl;
//   //   std::cerr<<"imu_last_data sample stop-1 time "<<imu_last_data[imu_last_data.size()-2].header.stamp<<std::endl;
//   // std::cerr<<"scanCallback sample start time "<<(*scan_msg).header.stamp<<std::endl;
//   // // **** if first scan, cache the tf from base to the scanner
//   //     }
//   std::cerr<<"scanCallback sample start time "<<(*scan_msg).header.stamp<<std::endl;
//   if (!initialized_)
//   {
//     createCache(scan_msg);    // caches the sin and cos of all angles
//     std::cerr<<"initialized_ is false**"<<std::endl;

//     // cache the static tf from base to laser
//     if (!getBaseToLaserTf(scan_msg->header.frame_id))
//     {
//       ROS_WARN("Skipping scan");
//       return;
//     }

//     laserScanToLDP(scan_msg, prev_ldp_scan_);
//     last_icp_time_ = scan_msg->header.stamp;
//     initialized_ = true;
//   }

//   LDP curr_ldp_scan;
//   laserScanToLDP(scan_msg, curr_ldp_scan);
//   processScan(curr_ldp_scan, scan_msg->header.stamp);
  
// }

// the new function scanCallback
void LaserScanMatcher::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  if(imu_data.size()<=2 || odom_data.size()<=2)
  {
    if(imu_data.size()<=2)
      ROS_WARN("wait imu data,only %i ",imu_data.size());
    if(odom_data.size()<=2)
    {
      ROS_WARN("wait odom data,only %i",odom_data.size());
    }
    return;
  }else
  {
    state_imu_ = !state_imu_;
    state_odom_ = !state_odom_;
    odom_last_data = odom_data;
    imu_last_data = imu_data;
  }
  
  
  if (!initialized_)
  {
    createCache(scan_msg);    // caches the sin and cos of all angles

    // cache the static tf from base to laser
    if (!getBaseToLaserTf(scan_msg->header.frame_id))
    {
      ROS_WARN("Skipping scan");
      return;
    }
    
    laserScanToLDP(scan_msg, prev_ldp_scan_);
    // if(!calibrateScan(prev_ldp_scan_))
    // {
    //   ROS_ERROR("Calibrate is error");
    //   return;
    // }
      
    last_icp_time_ = scan_msg->header.stamp;
    initialized_ = true;
    laser_last_data = *scan_msg;
    
  }
  laser_last_data = *scan_msg;

  LDP curr_ldp_scan;
  // laserScanToLDP(scan_msg, curr_ldp_scan);
  // processScan(curr_ldp_scan, scan_msg->header.stamp);
  laserScanToLDP_(laser_last_data, curr_ldp_scan);
  if(!calibrateScan(curr_ldp_scan))
  {
    ROS_ERROR("Calibrate is error");
    return;
  }
  processScan(curr_ldp_scan, scan_msg->header.stamp);
  // if(publish_laserscanpro_){
      
  //     laserscan_pro::LaserScanPro pub_pro;
  //     // double rangemax=0,rangemin=100,thetamax=0,thetamin=10;

  //     pub_pro.angle_increment = laser_last_data.angle_increment;
      
  //     pub_pro.header = laser_last_data.header;
  //     pub_pro.intensities = laser_last_data.intensities;
  //     pub_pro.ranges.resize(laser_last_data.ranges.size());
  //     pub_pro.theta.resize(laser_last_data.ranges.size());
      
  //     for (int h=0;h< laser_last_data.ranges.size();h++)
  //     {
  //       pub_pro.ranges[h] = curr_ldp_scan->readings[h];
  //       // rangemax = rangemax > pub_pro.ranges[h]? rangemax:pub_pro.ranges[h];
  //       // rangemin = rangemin < pub_pro.ranges[h]? rangemax:pub_pro.ranges[h];
  //     // }
  //     // for (int h=0;h< laser_last_data.ranges.size();h++)
  //     // {
  //       pub_pro.theta[h] = curr_ldp_scan->theta[h];
  //       // thetamax = thetamax > pub_pro.theta[h]? rangemax:pub_pro.theta[h];
  //       // thetamin = thetamin < pub_pro.theta[h]? rangemax:pub_pro.theta[h];
  //       // ROS_INFO("%i %.5f %.2f",h,pub_pro.theta[h],pub_pro.ranges[h]);
  //     }
  //     pub_pro.angle_max = laser_last_data.angle_max;
  //     pub_pro.angle_min = laser_last_data.angle_min;
  //     pub_pro.range_max = laser_last_data.range_max;
  //     pub_pro.range_min = laser_last_data.range_min;
  //     pub_pro.scan_time = laser_last_data.scan_time;
  //     pub_pro.time_increment = laser_last_data.time_increment;
  //     laserscanpro_publisher_.publish(pub_pro);
  //     flag ++;
  //     if(flag%10 == 0){
  //       ROS_ERROR("scanpro 10");
  //     }
  //   }
 
}
//use imu_last_data,odom_last_data to calibrate laser_last_data 

bool LaserScanMatcher::calibrateScan(LDP &last_ldp_scan)
{
  LDP last_scan_ldp = last_ldp_scan;
  //校正时间段 8ms一次
  int interpolation_time_duration = 8 * 1000;

  tf::Stamped<tf::Pose> frame_start_pose;
  tf::Stamped<tf::Pose> frame_mid_pose;
  tf::Stamped<tf::Pose> frame_insert_pose;
  tf::Stamped<tf::Pose> frame_base_pose;
  // tf::Stamped<tf::Pose> frame_end_pose;

  ros::Time startTime =  laser_last_data.header.stamp;
  // ros::Time endTime =  laser_last_data.header.stamp+ros::Duration(laser_last_data.time_increment*(laser_last_data.ranges.size()-1));
  double start_time = startTime.toSec()*1000*1000;
  // double end_time = endTime.toSec()*1000*1000;
  double inc_time = laser_last_data.time_increment*1000*1000;

  // if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0), tf_listener_)){
  //   ROS_WARN("base Pose,use odom_last_data[0]: %.1f %.1f",odom_last_data[0].pose.pose.position.x,
  //                                           odom_last_data[0].pose.pose.position.y);
  frame_start_pose.setOrigin(tf::Vector3(odom_last_data[0].pose.pose.position.x,
                                         odom_last_data[0].pose.pose.position.y, 0));
  // }

  //第一个点为基础，并获取基础点的坐标角度
  frame_base_pose = frame_start_pose;
  double base_x = frame_base_pose.getOrigin().x();
  double base_y = frame_base_pose.getOrigin().y();

  int start_index = 0;
  int cnt = 0;

  for(int i = 0;i<laser_last_data.ranges.size();i++){

    double mid_time = start_time + inc_time*(i-start_index);

    if((mid_time-start_time)>=interpolation_time_duration || (i==(laser_last_data.ranges.size()-1)))
    {
      cnt++;
      char flag_mid_end = 0;
      // if(!getLaserPose(frame_mid_pose,ros::Time(mid_time/1000000.0), tf_listener_))
      // {
        // ROS_WARN("end Pose: so start to use odom vector,TOTAL %i ", odom_last_data.size());
        for(int a=0;a<odom_last_data.size()-1;a++)
        {
          if(odom_last_data[a].header.stamp.toSec()<=(mid_time/1000000.0) 
             && odom_last_data[a+1].header.stamp.toSec()>(mid_time/1000000.0))
            {
              double x1=odom_last_data[a].pose.pose.position.x;
              double y1=odom_last_data[a].pose.pose.position.y;
              double y2=odom_last_data[a+1].pose.pose.position.y;
              double x2=odom_last_data[a+1].pose.pose.position.x;
              double t1=odom_last_data[a].header.stamp.toSec()*1000000;
              double t2=odom_last_data[a+1].header.stamp.toSec()*1000000;
              double xa=x1+(mid_time-t1)/(t2-t1)*(x2-x1);
              double ya=y1+(mid_time-t1)/(t2-t1)*(y2-y1);

              frame_mid_pose.setOrigin(tf::Vector3(xa,ya,0));
              // ROS_WARN("end Pose use odom_last_data %i : %.1f %.1f",a,frame_mid_pose.getOrigin().getX(),frame_mid_pose.getOrigin().getY());
              // ROS_WARN("end Pose: use odom_last_data :  ",frame_mid_pose.getOrigin().getX(),frame_mid_pose.getOrigin().getY());
              flag_mid_end = 1;
              break;
            }
            else
            {
              if(a == (odom_last_data.size()-1))
              {
                ROS_ERROR("end Pose:No data from mid date odom vector is availiable");
              }
            }
        }
        if(flag_mid_end==0)
        {
          frame_mid_pose.setOrigin(tf::Vector3(odom_last_data.back().pose.pose.position.x,odom_last_data.back().pose.pose.position.y,0));
        }
      // }
      // if(!getLaserPose(frame_insert_pose,ros::Time((mid_time+start_time)/2.0/1000000.0), tf_listener_)){
        // ROS_WARN("insert Pose: so start to use odom vector,TOTAL %i ", odom_last_data.size());
        for(int b=0;b<odom_last_data.size()-1;b++)
        {
          if(odom_last_data[b].header.stamp.toSec()<=((mid_time+start_time)/2.0/1000000.0) 
             && odom_last_data[b+1].header.stamp.toSec()>((mid_time+start_time)/2.0/1000000.0)){
              double x1=odom_last_data[b].pose.pose.position.x;
              double y1=odom_last_data[b].pose.pose.position.y;
              double y2=odom_last_data[b+1].pose.pose.position.y;
              double x2=odom_last_data[b+1].pose.pose.position.x;
              double t1=odom_last_data[b].header.stamp.toSec()*1000000;
              double t2=odom_last_data[b+1].header.stamp.toSec()*1000000;
              double x=x1+((mid_time+start_time)/2.0-t1)/(t2-t1)*(x2-x1);
              double y=y1+((mid_time+start_time)/2.0-t1)/(t2-t1)*(y2-y1);

              frame_insert_pose.setOrigin(tf::Vector3(x,y,0));
              // ROS_WARN("insert Pose: use odom_last_data : %.1f %.1f ",frame_insert_pose.getOrigin().getX(),frame_insert_pose.getOrigin().getY());
              // ROS_WARN("insert Pose: use odom_last_data %i : %.1f %.1f",b,frame_insert_pose.getOrigin().getX(),frame_insert_pose.getOrigin().getY());
              break;
            }
            else
            {
              if(b == (odom_last_data.size()-1))
              {
                ROS_ERROR("No insert Pose");
                return false;
              }
            }
        }
      // }

      int interp_count = i - start_index-1;

      //计算三个点的相应坐标
      double dx=0,dy=0,mx=0,my=0,ex=0,ey=0,ax=0,ay=0,Ax=0,Bx=0,Ay=0,By=0;
      dx=frame_start_pose.getOrigin().x();  dy=frame_start_pose.getOrigin().y();//t=0
      ex=frame_mid_pose.getOrigin().x();    ey=frame_mid_pose.getOrigin().y();//t=interp_count*inc_time
      mx=frame_insert_pose.getOrigin().x(); my=frame_insert_pose.getOrigin().y();//t=endt/2
      ex-=dx;mx-=dx;ey-=dy;my-=dy;
      //获得imu的值
      for(int z=0;z<imu_last_data.size();z++)
      {
        if(imu_last_data[z].header.stamp.toSec()<(start_time/1000000.0) 
           && imu_last_data[z+1].header.stamp.toSec()>(start_time/1000000.0))
            {ax=imu_last_data[z+1].linear_acceleration.x;ay=imu_last_data[z+1].linear_acceleration.y;}
      }
      if(ex==0)
        ax=0;
      if(ey==0)
        ay=0;
      

      double et;
      et = mid_time - start_time;
      et=et/1000;
      //x=1/6 * a * t^3 + A t^2 +Bt +C
      //A=-1/(t^2)*((4m-1/12*a*t^3)-2(e-1/6*a*t^3))= -1/(t^2) * (4m-2e+a/4*t^3)
      //B=-1/t*(-4m+a/12*t^3+e-a/6*t^3)            = -1/t     * (-4m+e-a/12*t^3)
      Ax = -1.0/pow(et,2)*(4*mx-2*ex+ax/4*pow(et,3));
      Bx = -1.0/et * (ex-4*mx-ax/12*pow(et,3));

      Ay = -1.0/pow(et,2)*(4*my-2*ey+ay/4*pow(et,3));
      By = -1.0/et * (ey-4*my-ay/12*pow(et,3));
	    //std::cerr<<"ax "<<ax<<" Ax "<<Ax<<" Bx "<<Bx<<std::endl;
	
      for(int j=1; j<=interp_count;j++)
      {
        double cx=0,cy=0,ct=inc_time*j,theta=0;
	      ct=ct/1000;
        //cx cy是此时相对于basex，basey的坐标,theta也是相对于base的角度
        cx=1/6*ax*pow(ct,3)+Ax*pow(ct,2)+Bx*ct;
        cy=1/6*ay*pow(ct,3)+Ay*pow(ct,2)+By*ct;

        //when there is a error in calibrate,then choose linear ;
        if(cx>mx || cx<0 || cy>my || cy<0){
          cx = ct/et * ex;
          cy = ct/et * ey;
        }
        

        // theta = (start_index+j)*laser_last_data.angle_increment+laser_last_data.angle_min;
        cx = cx + dx - base_x; cy = cy + dy - base_y;

        theta = last_ldp_scan->theta[start_index+j];
        cx+=laser_last_data.ranges[start_index+j]*cos(theta);
        cy+=laser_last_data.ranges[start_index+j]*sin(theta);
        //change reading and theta
		    //if (last_ldp_scan->readings[start_index+j] > 100){ROS_WARN("%i has overed 100",start_index+j);}

	      if (last_ldp_scan->readings[start_index+j] >= 0.15 && last_ldp_scan->readings[start_index+j] <= 12)
        {
	        //std::cerr<<"before  "<<last_ldp_scan->readings[start_index+j]<<" cx "<<cx<<" cy "<<cy<<std::endl;
          last_ldp_scan->readings[start_index+j] = sqrt(pow(cx,2)+pow(cy,2));
	        //std::cerr<<"after   "<<last_ldp_scan->readings[start_index+j]<<std::endl;std::cerr<<std::endl;
          last_ldp_scan->theta[start_index+j] = atan2(cy,cx);
          if(last_ldp_scan->readings[start_index+j]>100)
          {
            ROS_ERROR("dx:%f  ex:%f dy:%f ey:%f",dx,ex,dy,ey);
            ROS_ERROR("BEYOUND 100 cx:%f cy:%f \n ax:%f Ax:%f Bx:%f \n ay:%f Ay:%f By:%f", cx,cy,ax,Ax,Bx,ay,Ay,By);
          }
	      }
        // if (last_ldp_scan->readings[start_index+j] >= 12){
        //   last_ldp_scan->readings[start_index+j]=12;
        // }

      }

      start_time = mid_time;
      start_index = i;
      frame_start_pose = frame_mid_pose;
    }
  }//here, finish the change on readings and theta,and store them in LDP last_scan_ldp(&last_ldp_scan)
    double maxtheta=-5, mintheta=5,minrange=1000000,maxrange=0;
    for(int i = 0;i<laser_last_data.ranges.size();i++)
    {
      maxtheta = maxtheta > last_ldp_scan->theta[i]?maxtheta:last_ldp_scan->theta[i];
      mintheta = mintheta < last_ldp_scan->theta[i]?mintheta:last_ldp_scan->theta[i];
      maxrange = maxrange > last_ldp_scan->readings[i]?maxrange:last_ldp_scan->readings[i];
      minrange = minrange < last_ldp_scan->readings[i]?minrange:last_ldp_scan->readings[i];
    }
  laser_last_data.angle_max=maxtheta;
  laser_last_data.angle_min=mintheta;
  laser_last_data.range_max=maxrange;
  laser_last_data.range_min=minrange;
  
  return true;
}


bool LaserScanMatcher::getLaserPose(tf::Stamped<tf::Pose> &pose, ros::Time dt, tf::TransformListener& tf_)
{
  pose.setIdentity();
  // tf::Stamped<tf::Pose> robot_pose;
  tf::StampedTransform TransForm;
  // robot_pose.setIdentity();
  // robot_pose.frame_id_ = "base_link";
  // robot_pose.stamp_ = dt;   //设置为ros::Time()表示返回最近的转换关系

  try
  {
    // tf_listener_.waitForTransform("/odomini", "/base_link", dt, ros::Duration(1));
    // tf_listener_.lookupTransform ("/odomini", "/base_link", dt, TransForm);
    
    if(!tf_.waitForTransform("/odomini","/base_link", dt, ros::Duration(0.05)))             // 0.15s 的时间可以修改
    {
      ROS_WARN("LidarMotion Can not Wait Transform");
      return false;
    }
    // tf_.transformPose("/odomini", robot_pose, pose);
    // ROS_INFO("wait ok");
    tf_.lookupTransform ("/odomini", "/base_link", dt, TransForm);
    pose.setBasis(TransForm.getBasis());
    pose.setRotation(TransForm.getRotation());
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
    return false; 
  }
  return true;
}

void LaserScanMatcher::processScan(LDP& curr_ldp_scan, const ros::Time& time)
{
  ros::WallTime start = ros::WallTime::now();

  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the laser frame since the last scan
  // The computed correction is then propagated using the tf machinery

  prev_ldp_scan_->odometry[0] = 0.0;
  prev_ldp_scan_->odometry[1] = 0.0;
  prev_ldp_scan_->odometry[2] = 0.0;

  prev_ldp_scan_->estimate[0] = 0.0;
  prev_ldp_scan_->estimate[1] = 0.0;
  prev_ldp_scan_->estimate[2] = 0.0;

  prev_ldp_scan_->true_pose[0] = 0.0;
  prev_ldp_scan_->true_pose[1] = 0.0;
  prev_ldp_scan_->true_pose[2] = 0.0;

  input_.laser_ref  = prev_ldp_scan_;
  input_.laser_sens = curr_ldp_scan;

  // **** estimated change since last scan
  // 增量法（里程计获得x，y增量，imu获得角度增量） 可能此处getPrediction（）可以修改为所需函数

  double dt = (time - last_icp_time_).toSec();
  double pr_ch_x, pr_ch_y, pr_ch_a;
  getPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);
  pr_ch_a = 0;
  // the predicted change of the laser's position, in the fixed frame

  tf::Transform pr_ch;
  createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, pr_ch);

  // account for the change since the last kf, in the fixed frame

  pr_ch = pr_ch * (f2b_ * f2b_kf_.inverse());

  // the predicted change of the laser's position, in the laser frame

  tf::Transform pr_ch_l;
  pr_ch_l = laser_to_base_ * f2b_.inverse() * pr_ch * f2b_ * base_to_laser_ ;

  input_.first_guess[0] = pr_ch_l.getOrigin().getX();
  input_.first_guess[1] = pr_ch_l.getOrigin().getY();
  input_.first_guess[2] = tf::getYaw(pr_ch_l.getRotation());

  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  // *** scan match - using point to line icp from CSM

  sm_icp(&input_, &output_);
  tf::Transform corr_ch;

  if (output_.valid)
  {

    // the correction of the laser's position, in the laser frame
    tf::Transform corr_ch_l;
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

    // the correction of the base's position, in the base frame
    corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;

    // update the pose in the world frame
    f2b_ = f2b_kf_ * corr_ch;

    // **** publish
    if (publish_tf_)
    {
      tf::StampedTransform transform_msg (f2b_, time, "odom", "base_link");
      tf_broadcaster_.sendTransform (transform_msg);
      // ROS_WARN("%.2f %.2f",f2b_.getOrigin().getX(),f2b_.getOrigin().getY());
      // ROS_WARN("%.2f %.2f %.2f %.2f",f2b_.getRotation().getW(),f2b_.getRotation().getX(),
      //                                 f2b_.getRotation().getY(),f2b_.getRotation().getZ());

    }
    if(publish_laserscanpro_){
      
      laserscan_pro::LaserScanPro pub_pro;
      // double rangemax=0,rangemin=100,thetamax=0,thetamin=10;

      pub_pro.angle_increment = laser_last_data.angle_increment;
      
      pub_pro.header = laser_last_data.header;
      pub_pro.intensities = laser_last_data.intensities;
      pub_pro.ranges.resize(laser_last_data.ranges.size());
      pub_pro.theta.resize(laser_last_data.ranges.size());
      
      for (int h=0;h< laser_last_data.ranges.size();h++)
      {
        pub_pro.ranges[h] = curr_ldp_scan->readings[h];
        // rangemax = rangemax > pub_pro.ranges[h]? rangemax:pub_pro.ranges[h];
        // rangemin = rangemin < pub_pro.ranges[h]? rangemax:pub_pro.ranges[h];
      // }
      // for (int h=0;h< laser_last_data.ranges.size();h++)
      // {
        pub_pro.theta[h] = curr_ldp_scan->theta[h];
        // thetamax = thetamax > pub_pro.theta[h]? rangemax:pub_pro.theta[h];
        // thetamin = thetamin < pub_pro.theta[h]? rangemax:pub_pro.theta[h];
        // ROS_INFO("%i %.5f %.2f",h,pub_pro.theta[h],pub_pro.ranges[h]);
      }
      pub_pro.angle_max = laser_last_data.angle_max;
      pub_pro.angle_min = laser_last_data.angle_min;
      pub_pro.range_max = laser_last_data.range_max;
      pub_pro.range_min = laser_last_data.range_min;
      pub_pro.scan_time = laser_last_data.scan_time;
      pub_pro.time_increment = laser_last_data.time_increment;
      laserscanpro_publisher_.publish(pub_pro);
      flag ++;
      if(flag%10 == 0){
        ROS_ERROR("scanpro 10");
      }
    }
    // if(publish_laserscanpro_){
      
    //   laserscan_pro::LaserScanPro pub_pro;
    //   double rangemax=0,rangemin=100,thetamax=0,thetamin=10;

    //   pub_pro.angle_increment = laser_last_data.angle_increment;
      
    //   pub_pro.header = laser_last_data.header;
    //   pub_pro.intensities = laser_last_data.intensities;
    //   pub_pro.ranges.resize(laser_last_data.ranges.size());
    //   pub_pro.theta.resize(laser_last_data.ranges.size());
      
    //   for (int h=0;h< laser_last_data.ranges.size();h++)
    //   {
    //     pub_pro.ranges[h] = curr_ldp_scan->readings[h];
    //     rangemax = rangemax > pub_pro.ranges[h]? rangemax:pub_pro.ranges[h];
    //     rangemin = rangemin < pub_pro.ranges[h]? rangemax:pub_pro.ranges[h];
    //   // }
    //   // for (int h=0;h< laser_last_data.ranges.size();h++)
    //   // {
    //     pub_pro.theta[h] = curr_ldp_scan->theta[h];
    //     thetamax = thetamax > pub_pro.theta[h]? rangemax:pub_pro.theta[h];
    //     thetamin = thetamin < pub_pro.theta[h]? rangemax:pub_pro.theta[h];
    //     // ROS_INFO("%i %.5f %.2f",h,pub_pro.theta[h],pub_pro.ranges[h]);
    //   }
    //   pub_pro.angle_max = thetamax;
    //   pub_pro.angle_min = thetamin;
    //   pub_pro.range_max = rangemax;
    //   pub_pro.range_min = rangemin;
    //   pub_pro.scan_time = laser_last_data.scan_time;
    //   pub_pro.time_increment = laser_last_data.time_increment;
    //   laserscanpro_publisher_.publish(pub_pro);
    //   flag ++;
    //   if(flag%10 == 0){
    //     ROS_ERROR("scanpro 10");
    //   }
    // }
    
    if (publish_pose_)
    {
      // unstamped Pose2D message
      geometry_msgs::Pose2D::Ptr pose_msg;
      pose_msg = boost::make_shared<geometry_msgs::Pose2D>();
      pose_msg->x = f2b_.getOrigin().getX();
      pose_msg->y = f2b_.getOrigin().getY();
      pose_msg->theta = tf::getYaw(f2b_.getRotation());
      pose_publisher_.publish(pose_msg);
    }
    if (publish_pose_stamped_)
    {
      // stamped Pose message
      geometry_msgs::PoseStamped::Ptr pose_stamped_msg;
      pose_stamped_msg = boost::make_shared<geometry_msgs::PoseStamped>();

      pose_stamped_msg->header.stamp    = time;
      pose_stamped_msg->header.frame_id = fixed_frame_;

      tf::poseTFToMsg(f2b_, pose_stamped_msg->pose);

      pose_stamped_publisher_.publish(pose_stamped_msg);
    }
    if (publish_pose_with_covariance_)
    {
      // unstamped PoseWithCovariance message
      geometry_msgs::PoseWithCovariance::Ptr pose_with_covariance_msg;
      pose_with_covariance_msg = boost::make_shared<geometry_msgs::PoseWithCovariance>();
      tf::poseTFToMsg(f2b_, pose_with_covariance_msg->pose);

      if (input_.do_compute_covariance)
      {
        pose_with_covariance_msg->covariance = boost::assign::list_of
          (gsl_matrix_get(output_.cov_x_m, 0, 0)) (0)  (0)  (0)  (0)  (0)
          (0)  (gsl_matrix_get(output_.cov_x_m, 0, 1)) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (gsl_matrix_get(output_.cov_x_m, 0, 2));
      }
      else
      {
        pose_with_covariance_msg->covariance = boost::assign::list_of
          (static_cast<double>(position_covariance_[0])) (0)  (0)  (0)  (0)  (0)
          (0)  (static_cast<double>(position_covariance_[1])) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[2]));
      }

      pose_with_covariance_publisher_.publish(pose_with_covariance_msg);
    }
    if (publish_pose_with_covariance_stamped_)
    {
      // stamped Pose message
      geometry_msgs::PoseWithCovarianceStamped::Ptr pose_with_covariance_stamped_msg;
      pose_with_covariance_stamped_msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

      pose_with_covariance_stamped_msg->header.stamp    = time;
      pose_with_covariance_stamped_msg->header.frame_id = fixed_frame_;

      tf::poseTFToMsg(f2b_, pose_with_covariance_stamped_msg->pose.pose);

      if (input_.do_compute_covariance)
      {
        pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of
          (gsl_matrix_get(output_.cov_x_m, 0, 0)) (0)  (0)  (0)  (0)  (0)
          (0)  (gsl_matrix_get(output_.cov_x_m, 0, 1)) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (gsl_matrix_get(output_.cov_x_m, 0, 2));
      }
      else
      {
        pose_with_covariance_stamped_msg->pose.covariance = boost::assign::list_of
          (static_cast<double>(position_covariance_[0])) (0)  (0)  (0)  (0)  (0)
          (0)  (static_cast<double>(position_covariance_[1])) (0)  (0)  (0)  (0)
          (0)  (0)  (static_cast<double>(position_covariance_[2])) (0)  (0)  (0)
          (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[0])) (0)  (0)
          (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[1])) (0)
          (0)  (0)  (0)  (0)  (0)  (static_cast<double>(orientation_covariance_[2]));
      }

      pose_with_covariance_stamped_publisher_.publish(pose_with_covariance_stamped_msg);
    }


  }
  else
  {
    corr_ch.setIdentity();
    ROS_WARN("Error in scan matching");
  }

  // **** swap old and new

  if (newKeyframeNeeded(corr_ch))
  {
    // generate a keyframe
    ld_free(prev_ldp_scan_);
    prev_ldp_scan_ = curr_ldp_scan;
    f2b_kf_ = f2b_;
  }
  else
  {
    ld_free(curr_ldp_scan);
  }

  last_icp_time_ = time;

  // **** statistics

  double dur = (ros::WallTime::now() - start).toSec() * 1e3;
  ROS_DEBUG("Scan matcher total duration: %.1f ms", dur);
}

bool LaserScanMatcher::newKeyframeNeeded(const tf::Transform& d)
{
  if (fabs(tf::getYaw(d.getRotation())) > kf_dist_angular_) return true;

  double x = d.getOrigin().getX();
  double y = d.getOrigin().getY();
  if (x*x + y*y > kf_dist_linear_sq_) return true;

  return false;
}

void LaserScanMatcher::PointCloudToLDP(const PointCloudT::ConstPtr& cloud,
                                             LDP& ldp)
{
  double max_d2 = cloud_res_ * cloud_res_;

  PointCloudT cloud_f;

  cloud_f.points.push_back(cloud->points[0]);

  for (unsigned int i = 1; i < cloud->points.size(); ++i)
  {
    const PointT& pa = cloud_f.points[cloud_f.points.size() - 1];
    const PointT& pb = cloud->points[i];

    double dx = pa.x - pb.x;
    double dy = pa.y - pb.y;
    double d2 = dx*dx + dy*dy;

    if (d2 > max_d2)
    {
      cloud_f.points.push_back(pb);
    }
  }

  unsigned int n = cloud_f.points.size();

  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame
    if (is_nan(cloud_f.points[i].x) || is_nan(cloud_f.points[i].y))
    {
      ROS_WARN("Laser Scan Matcher: Cloud input contains NaN values. \
                Please use a filtered cloud input.");
    }
    else
    {
      double r = sqrt(cloud_f.points[i].x * cloud_f.points[i].x +
                      cloud_f.points[i].y * cloud_f.points[i].y);

      if (r > cloud_range_min_ && r < cloud_range_max_)
      {
        ldp->valid[i] = 1;
        ldp->readings[i] = r;
      }
      else
      {
        ldp->valid[i] = 0;
        ldp->readings[i] = -1;  // for invalid range
      }
    }

    ldp->theta[i] = atan2(cloud_f.points[i].y, cloud_f.points[i].x);
    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}
void LaserScanMatcher::laserScanToLDP_(sensor_msgs::LaserScan& scan_msg,
                                            LDP& ldp)
{
  unsigned int n = scan_msg.ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = scan_msg.ranges[i];

    if (r > scan_msg.range_min && r < scan_msg.range_max)
    {
      // fill in laser scan data

      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }

    ldp->theta[i]    = scan_msg.angle_min + i * scan_msg.angle_increment;

    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}
void LaserScanMatcher::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                            LDP& ldp)
{
  unsigned int n = scan_msg->ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = scan_msg->ranges[i];

    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
      // fill in laser scan data

      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }

    ldp->theta[i]    = scan_msg->angle_min + i * scan_msg->angle_increment;

    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void LaserScanMatcher::createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  a_cos_.clear();
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }

  input_.min_reading = scan_msg->range_min;
  input_.max_reading = scan_msg->range_max;
}
// try to get from source frame (base_link) to frame_id 
bool LaserScanMatcher::getBaseToLaserTf (const std::string& frame_id)
{
  ros::Time t = ros::Time::now();

  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener_.waitForTransform(
      base_frame_, frame_id, t, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, frame_id, t, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
    return false;
  }
  base_to_laser_ = base_to_laser_tf;
  laser_to_base_ = base_to_laser_.inverse();

  return true;
}

// returns the predicted change in pose (in fixed frame)
// since the last time we did icp
void LaserScanMatcher::getPrediction(double& pr_ch_x, double& pr_ch_y,
                                     double& pr_ch_a, double dt)
{
  boost::mutex::scoped_lock(mutex_);

  // **** base case - no input available, use zero-motion model
  pr_ch_x = 0.0;
  pr_ch_y = 0.0;
  pr_ch_a = 0.0;

  // **** use velocity (for example from ab-filter)
  if (use_vel_)
  {
    pr_ch_x = dt * latest_vel_msg_.linear.x;
    pr_ch_y = dt * latest_vel_msg_.linear.y;
    pr_ch_a = dt * latest_vel_msg_.angular.z;

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;
  }

  // **** use wheel odometry
  if (use_odom_ && received_odom_)
  {
    pr_ch_x = latest_odom_msg_.pose.pose.position.x -
              last_used_odom_msg_.pose.pose.position.x;

    pr_ch_y = latest_odom_msg_.pose.pose.position.y -
              last_used_odom_msg_.pose.pose.position.y;

    pr_ch_a = tf::getYaw(latest_odom_msg_.pose.pose.orientation) -
              tf::getYaw(last_used_odom_msg_.pose.pose.orientation);

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;

    last_used_odom_msg_ = latest_odom_msg_;
  }

  // **** use imu
  if (use_imu_ && received_imu_)
  {
    pr_ch_a = tf::getYaw(latest_imu_msg_.orientation) -
              tf::getYaw(last_used_imu_msg_.orientation);

    if      (pr_ch_a >= M_PI) pr_ch_a -= 2.0 * M_PI;
    else if (pr_ch_a < -M_PI) pr_ch_a += 2.0 * M_PI;

    last_used_imu_msg_ = latest_imu_msg_;
  }
}

void LaserScanMatcher::createTfFromXYTheta(
  double x, double y, double theta, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

} // namespace scan_tools
