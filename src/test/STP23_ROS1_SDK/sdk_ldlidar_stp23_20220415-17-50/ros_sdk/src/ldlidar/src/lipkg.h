/**
 * @file lipkg.h
 * @author LDRobot (marketing1@ldrobot.com)
 * @brief  LiDAR data protocol processing App
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

#ifndef __LIPKG_H
#define __LIPKG_H

#include <sensor_msgs/LaserScan.h>
#include <stdint.h>

#include <array>
#include <iostream>
#include <vector>

#include "pointdata.h"

enum class LDVersion {
  SIX,
  SIX_P,
  STP23,
};

enum {
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t temperature;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

class LiPkg {
 public:
  LiPkg(std::string frame_id, LDVersion version);
  // get Lidar temperature 
  double GetTemperature(void) { return temperature_; } 
  // get time stamp of the packet
  uint16_t GetTimestamp(void) { return timestamp_; }  
  // a packet is ready                                              
  bool IsPkgReady(void) { return is_pkg_ready_; }  
  // Get lidar data frame ready flag  
  bool IsFrameReady(void) { return is_frame_ready_; }  
  // Lidar data frame readiness flag reset
  void ResetFrameReady(void) { is_frame_ready_ = false; }
  // the number of errors in parser process of lidar data frame
  long GetErrorTimes(void) { return error_times_; } 
  // Get original Lidar data package
  const std::array<PointData, POINT_PER_PACK>& GetPkgData(void);  
  // parse single packet
  bool AnalysisOne(uint8_t byte);
  bool Parse(const uint8_t* data, long len);  
  // combine stantard data into data frames and calibrate
  bool AssemblePacket();  
  Points2D GetData() {return lidar_frame_data_;};
 private:
  int point_frequence_;
  LDVersion version_;
  std::string frame_id_;
  uint16_t timestamp_;
  double temperature_;
  float angle_increment_;
  long error_times_;
  LiDARFrameTypeDef pkg;
  std::vector<uint8_t> data_tmp_;
  std::array<PointData, POINT_PER_PACK> one_pkg_;
  std::vector<PointData> frame_tmp_;
  Points2D lidar_frame_data_;
  bool is_pkg_ready_;
  bool is_frame_ready_;
};

#endif  //__LIPKG_H

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
