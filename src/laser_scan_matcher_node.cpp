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

// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/time_synchronizer.h>

// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/LaserScan.h>
// #include <message_filters/time_synchronizer.h>
// void mbinecallback(const sensor_msgs::ImuConstPtr& scan_imu,const sensor_msgs::LaserScanConstPtr& scan_laser){
//   sensor_msgs::LaserScan laser=*scan_laser;
//   sensor_msgs::Imu imu=*scan_imu;
//   std::cerr<<"num of laser reading is "<<laser.ranges.size()<<std::endl;
//   // std::cerr<<"times: "<<num_imu<<std::endl;
//   // num_imu++;
//   std::cerr<<"num of imu data is "<<imu.header.frame_id<<std::endl;

// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LaserScanMatcher");
  ros::NodeHandle nh;

  // message_filters::Subscriber<sensor_msgs::Imu> image_sub(nh, "imu", 1);             // topic1 输入
  // message_filters::Subscriber<sensor_msgs::LaserScan> info_sub(nh, "scan", 1);   // topic2 输入
  // message_filters::TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::LaserScan> sync(image_sub, info_sub, 10);       // 同步
  // sync.registerCallback(boost::bind(&mbinecallback, _1, _2));                   // 回调

  ros::NodeHandle nh_private("~");
  scan_tools::LaserScanMatcher laser_scan_matcher(nh, nh_private);
  //加入odom写为3 
  // ros::MultiThreadedSpinner s(4);  //多线程
  // ros::spin(s);
  ros::spin();
  return 0;
}
