/**
 * @file main.cpp
 * @author LDRobot (marketing1@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * sold by Shenzhen LDROBOT Co., LTD    
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <stdio.h>

#include <iostream>
#include <string>

#include "cmd_interface_linux.h"
#include "lipkg.h"
#include "ldlidar/STP.h"
#include "std_msgs/Float32.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ldlidar_pub_node");
  ros::NodeHandle nh;  // create a ROS Node
  ros::NodeHandle nh_private("~");
  std::string product_name;
  std::string topic_name;
  std::string port_name;
  std::string frame_id;

  nh_private.getParam("product_name", product_name);
  nh_private.getParam("topic_name", topic_name);
  nh_private.getParam("port_name", port_name);
  nh_private.getParam("frame_id", frame_id);
  
  ROS_INFO_STREAM("[ldrobot] SDK Pack Version is " << "v1.0.0");

  LiPkg *lidar = nullptr;

  if (product_name == "LDLiDAR_STP23") {
    lidar = new LiPkg(frame_id, LDVersion::STP23);
  } else {
    ROS_ERROR("[ldrobot] Error, input param <product_name> is fail!!");
    exit(EXIT_FAILURE);
  }

  CmdInterfaceLinux cmd_port;

  if (port_name.empty()) {
    ROS_ERROR("[ldrobot] Error, input param <port_name> is empty");
    exit(EXIT_FAILURE);
  } else {
    cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
      if (lidar->Parse((uint8_t *)byte, len)) {
        lidar->AssemblePacket();
      }
    });
  }

  if (cmd_port.Open(port_name)) {
    ROS_INFO_STREAM("[ldrobot] open " << product_name << " device  " << port_name  << " success!");
  } else {
    ROS_INFO_STREAM("[ldrobot] open " << product_name << " device  " << port_name << " fail!");
    exit(EXIT_FAILURE);
  }
    
  ros::Publisher lidar_pub = nh.advertise<ldlidar::STP>(topic_name, 1000);  // create a ROS topic
  ros::Publisher distance_pub = nh.advertise<std_msgs::Float32>("/ldlidar_distance", 1000); 
  
  ros::Rate r(100); //100hz
  Points2D laser_data;
  ldlidar::STP stp_;
  std_msgs::Float32 distance_msg;  // 创建距离消息对象
  
  while (ros::ok()) {
    if (lidar->IsFrameReady()) {
      laser_data = lidar->GetData();
      // std::cout<<laser_data.size()<<std::endl;
      for (auto point : laser_data) {
          stp_.header.stamp = ros::Time::now();
          stp_.header.frame_id = frame_id;
          stp_.distance = floor((point.distance/1000.f)*10000+0.5)/10000;
          std::cout << "[STP23] distance(m): " << stp_.distance << std::endl;
          stp_.intensity = (int)point.intensity;
          std::cout << "[STP23] intensity: " << stp_.intensity << std::endl;
          std::cout << "----------------------------" << std::endl;
          
          // 设置并发布距离消息
          distance_msg.data = stp_.distance;
          distance_pub.publish(distance_msg);
          
          lidar_pub.publish(stp_);  // Fixed Frame:  lidar_frame
          ros::spinOnce();
          r.sleep();
      }
      lidar->ResetFrameReady();
#if 0 
      sensor_msgs::LaserScan data = lidar->GetLaserScan();
      unsigned int lens = (data.angle_max - data.angle_min) / data.angle_increment;  
      for (int i = 0; i < lens; i++)
      {
        std::cout << "[ldrobot] range: " <<  data.ranges[i] << " " 
                  << "intensites: " <<  data.intensities[i] << std::endl;
      }
      std::cout << "----------------------------" << std::endl;
#endif
    }
  }
  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
