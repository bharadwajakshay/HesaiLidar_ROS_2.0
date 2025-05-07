/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted 
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions 
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and 
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of 
   other contributors maybe used to endorse or promote products derived from this software without 
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE 
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF 
  SUCHDAMAGE.
************************************************************************************************/

/*
 * File: source_driver_ros1.hpp
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: Source Driver for ROS1
 * Created on June 12, 2023, 10:46 AM
 */

 #pragma once
 #include <ros/ros.h>
 #include "std_msgs/UInt8MultiArray.h"
 #include <sensor_msgs/point_cloud2_iterator.h>
 #include "hesai_ros_driver/UdpFrame.h"
 #include "hesai_ros_driver/UdpPacket.h"
 #include "hesai_ros_driver/LossPacket.h"
 #include "hesai_ros_driver/Ptp.h"
 #include "hesai_ros_driver/Firetime.h"
 #include <sensor_msgs/Imu.h>
 #include <boost/thread.hpp>
 #include "source_drive_common.hpp"
 #include <vector>
 #include <iomanip> // For std::hex
 
 class SourceDriver
 {
 public:
   typedef std::shared_ptr<SourceDriver> Ptr;
   // Initialize some necessary configuration parameters, create ROS nodes, and register callback functions
   virtual void Init(const YAML::Node& config);
   // Start working
   virtual void Start();
   // Stop working
   virtual void Stop();
   virtual ~SourceDriver();
   SourceDriver(SourceType src_type) {};
   void SpinRos1() {
     ros::MultiThreadedSpinner spinner(2); 
     spinner.spin();
   }
   #ifdef __CUDACC__
     std::shared_ptr<HesaiLidarSdkGpu<LidarPointXYZIRT>> driver_ptr_;
   #else
     std::shared_ptr<HesaiLidarSdk<LidarPointXYZIRT>> driver_ptr_;
   #endif
 protected:
   void PublishRawPacket(const hesai::lidar::UdpPacket& sdk_packet); // Use the SDK namespace
   // Save Correction file subscribed by "ros_recv_correction_topic"
   void RecieveCorrection(const std_msgs::UInt8MultiArray& msg);
   // Save packets subscribed by 'ros_recv_packet_topic'
   void RecievePacket(const hesai_ros_driver::UdpFrame& msg);
   // Used to publish point clouds through 'ros_send_point_cloud_topic'
   void SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg);
   // Used to publish the original pcake through 'ros_send_packet_topic'
   void SendPacket(const UdpFrame_t&  ros_msg, double);
   // Used to publish the Correction file through 'ros_send_correction_topic'
   void SendCorrection(const u8Array_t& msg);
   // Used to publish the Packet loss condition
   void SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
   // Used to publish the Packet loss condition
   void SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
   // Used to publish the firetime correction 
   void SendFiretime(const double *firetime_correction_);
   // Used to publish the imu packet
   void SendImuConfig(const LidarImuData& msg);
   // Convert ptp lock offset, status into ROS message
   hesai_ros_driver::Ptp ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
   // Convert packet loss condition into ROS message
   hesai_ros_driver::LossPacket ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
   // Convert correction string into ROS messages
   std_msgs::UInt8MultiArray ToRosMsg(const u8Array_t& correction_string);
   // Convert point clouds into ROS messages
   sensor_msgs::PointCloud2 ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id);
   // Convert packets into ROS messages
   hesai_ros_driver::UdpFrame ToRosMsg(const UdpFrame_t& ros_msg, double timestamp);
   // Convert double[512] to float64[512]
   hesai_ros_driver::Firetime ToRosMsg(const double *firetime_correction_);
   // Convert imu, imu into ROS message
   sensor_msgs::Imu ToRosMsg(const LidarImuData& firetime_correction_);
   // publish point
   std::shared_ptr<ros::NodeHandle> nh_;
   ros::Publisher pub_;
   std::string frame_id_;
   // publish packet 
   ros::Publisher pkt_pub_;
   ros::Publisher raw_pkt_pub_;
   // packet sub
   ros::Subscriber pkt_sub_;
   //spin thread while recieve data from ROS topic
   boost::thread* subscription_spin_thread_;
 
   ros::Publisher crt_pub_;
   ros::Publisher firetime_pub_;
   ros::Publisher loss_pub_;
   ros::Publisher ptp_pub_;
   ros::Subscriber crt_sub_;
   ros::Publisher imu_pub_;
 };
 
 // Implementation (inside the .hpp for inline, or in a .cc file)
 inline void SourceDriver::PublishRawPacket(const hesai::lidar::UdpPacket& sdk_packet)
 {
     // 1. Check if the publisher is valid
     #ifdef ROS_FOUND
         if (!raw_pkt_pub_) {
             // ROS_ERROR_ONCE("Raw packet publisher is not initialized!"); // Log error once if publisher is invalid
             return;
         }
     #elif ROS2_FOUND
         if (!raw_pkt_pub_) {
             // RCLCPP_ERROR_ONCE(node_ptr_->get_logger(), "Raw packet publisher is not initialized!"); // Log error once if publisher is invalid
             return;
         }
     #endif
 
 
     // 2. Log that the function is called and the incoming packet size
     #ifdef ROS_FOUND
         // ROS_INFO("PublishRawPacket called. SDK Packet Size: %u", sdk_packet.packet_len);
     #elif ROS2_FOUND
         // RCLCPP_INFO(node_ptr_->get_logger(), "PublishRawPacket called. SDK Packet Size: %u", sdk_packet.packet_len);
     #endif
 
     // 3. Check if the incoming SDK packet actually has data
     if (sdk_packet.packet_len == 0) {
         #ifdef ROS_FOUND
             ROS_WARN("Received SDK packet with size 0 in PublishRawPacket. Skipping publish.");
         #elif ROS2_FOUND
             // RCLCPP_WARN(node_ptr_->get_logger(), "Received SDK packet with size 0 in PublishRawPacket. Skipping publish.");
         #endif
         return; // Don't publish empty packets
     }
 
 
     // Create the ROS UdpPacket message
     #ifdef ROS_FOUND
         hesai_ros_driver::UdpPacket ros_packet_msg;
         // Use sdk_packet.recv_timestamp for the header
         ros_packet_msg.header.stamp = ros::Time().fromSec(static_cast<double>(sdk_packet.recv_timestamp) / 1e6);
     #elif ROS2_FOUND
         hesai_ros_driver::msg::UdpPacket ros_packet_msg;
         // Use sdk_packet.recv_timestamp for the header
         rclcpp::Time packet_stamp;
         auto sec = static_cast<uint64_t>(floor(static_cast<double>(sdk_packet.recv_timestamp) / 1e6));
         if (sec > std::numeric_limits<int32_t>::max()) {
              // RCLCPP_WARN_ONCE(node_ptr_->get_logger(), "Timestamp exceeds INT32_MAX (%lu), potential overflow. Clamping.", sec);
              // Clamp or handle appropriately if needed, here we just proceed
              packet_stamp = rclcpp::Time(std::numeric_limits<int32_t>::max(), 999999999); // Example clamping
         } else {
              packet_stamp = rclcpp::Time(sdk_packet.recv_timestamp * 1000ULL); // Convert us to ns
         }
         ros_packet_msg.header.stamp = packet_stamp;
     #endif
 
     // Populate Header and Data
     ros_packet_msg.header.frame_id = frame_id_;
     ros_packet_msg.size = sdk_packet.packet_len;
     ros_packet_msg.data.resize(sdk_packet.packet_len);
     memcpy(&ros_packet_msg.data[0], &sdk_packet.buffer[0], sdk_packet.packet_len);
 
     // 4. Log details just before publishing
     #ifdef ROS_FOUND
         // ROS_INFO("Publishing raw packet: Stamp=%f, FrameID=%s, Size=%u",
         //          ros_packet_msg.header.stamp.toSec(),
         //          ros_packet_msg.header.frame_id.c_str(),
         //          ros_packet_msg.size);
         // // Optionally log first few bytes
         // std::stringstream ss;
         // for(size_t i = 0; i < std::min((size_t)10, (size_t)ros_packet_msg.size); ++i) {
         //     ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << (int)ros_packet_msg.data[i] << " ";
         // }
         // ROS_INFO("  Data prefix: %s", ss.str().c_str());
 
     #elif ROS2_FOUND
         // RCLCPP_INFO(node_ptr_->get_logger(), "Publishing raw packet: Stamp=%f, FrameID=%s, Size=%u",
         //             rclcpp::Time(ros_packet_msg.header.stamp).seconds(),
         //             ros_packet_msg.header.frame_id.c_str(),
         //             ros_packet_msg.size);
         // // Optionally log first few bytes
         // std::stringstream ss;
         // for(size_t i = 0; i < std::min((size_t)10, (size_t)ros_packet_msg.size); ++i) {
         //     ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << (int)ros_packet_msg.data[i] << " ";
         // }
         // RCLCPP_INFO(node_ptr_->get_logger(), "  Data prefix: %s", ss.str().c_str());
     #endif
 
 
     // Publish
     #ifdef ROS_FOUND
         raw_pkt_pub_.publish(ros_packet_msg);
     #elif ROS2_FOUND
         raw_pkt_pub_->publish(ros_packet_msg);
     #endif
 }
 inline void SourceDriver::Init(const YAML::Node& config)
 {
   
   DriverParam driver_param;
   DriveYamlParam yaml_param;
   yaml_param.GetDriveYamlParam(config, driver_param);
   frame_id_ = driver_param.input_param.frame_id;
 
   nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
   if (driver_param.input_param.send_point_cloud_ros) {
     pub_ = nh_->advertise<sensor_msgs::PointCloud2>(driver_param.input_param.ros_send_point_topic, 10);
   }
   
   if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
     loss_pub_ = nh_->advertise<hesai_ros_driver::LossPacket>(driver_param.input_param.ros_send_packet_loss_topic, 10);
   } 
 
   if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
     if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
       ptp_pub_ = nh_->advertise<hesai_ros_driver::Ptp>(driver_param.input_param.ros_send_ptp_topic, 10);
     } 
 
     if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
       crt_pub_ = nh_->advertise<std_msgs::UInt8MultiArray>(driver_param.input_param.ros_send_correction_topic, 10);
     } 
   }
   if (! driver_param.input_param.firetimes_path.empty() ) {
     if (driver_param.input_param.ros_send_firetime_topic != NULL_TOPIC) {
       firetime_pub_ = nh_->advertise<hesai_ros_driver::Firetime>(driver_param.input_param.ros_send_firetime_topic, 10);
     } 
   }
 
   if (driver_param.input_param.send_packet_ros && driver_param.input_param.source_type != DATA_FROM_ROS_PACKET) {
     pkt_pub_ = nh_->advertise<hesai_ros_driver::UdpFrame>(driver_param.input_param.ros_send_packet_topic, 10);
   }
 
     // Initialize the new individual packet publisher
   if (driver_param.input_param.send_raw_packet_ros) {
     raw_pkt_pub_ = nh_->advertise<hesai_ros_driver::UdpPacket>(driver_param.input_param.ros_send_raw_packet_topic, 500); // Higher queue size might be needed   
   }
 
   if (driver_param.input_param.source_type == DATA_FROM_ROS_PACKET) {
     pkt_sub_ = nh_->subscribe(driver_param.input_param.ros_recv_packet_topic, 100, &SourceDriver::RecievePacket, this);
 
     if (driver_param.input_param.ros_recv_correction_topic != NULL_TOPIC) {
       crt_sub_ = nh_->subscribe(driver_param.input_param.ros_recv_correction_topic, 10, &SourceDriver::RecieveCorrection, this);
     }
 
     driver_param.decoder_param.enable_udp_thread = false;
     subscription_spin_thread_ = new boost::thread(boost::bind(&SourceDriver::SpinRos1,this));
   }
   
   #ifdef __CUDACC__
     driver_ptr_.reset(new HesaiLidarSdkGpu<LidarPointXYZIRT>());
     driver_param.decoder_param.enable_parser_thread = false;
   #else
     driver_ptr_.reset(new HesaiLidarSdk<LidarPointXYZIRT>());
     driver_param.decoder_param.enable_parser_thread = true;
   #endif
   driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPointCloud, this, std::placeholders::_1));
   imu_pub_ = nh_->advertise<sensor_msgs::Imu>(driver_param.input_param.ros_send_imu_topic, 10);
   driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendImuConfig, this, std::placeholders::_1));
   if(driver_param.input_param.send_packet_ros && driver_param.input_param.source_type != DATA_FROM_ROS_PACKET){
     driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacket, this, std::placeholders::_1, std::placeholders::_2)) ;
   }
   if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
     driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacketLoss, this, std::placeholders::_1, std::placeholders::_2));
   }
   if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
     if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
       driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendCorrection, this, std::placeholders::_1));
     }
     if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
       driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPTP, this, std::placeholders::_1, std::placeholders::_2));
     }
   }
 
   if (!driver_ptr_->Init(driver_param))
   {
     std::cout << "Driver Initialize Error...." << std::endl;
     exit(-1);
   }
 
   if (driver_param.input_param.send_raw_packet_ros) {
     driver_ptr_->lidar_ptr_->RegRecvRawPacketCallback(std::bind(&SourceDriver::PublishRawPacket, this, std::placeholders::_1));
   }
 }
 
 inline void SourceDriver::Start()
 {
   driver_ptr_->Start();
 }
 
 inline SourceDriver::~SourceDriver()
 {
   Stop();
 }
 
 inline void SourceDriver::Stop()
 {
   driver_ptr_->Stop();
 }
 
 inline void SourceDriver::SendPacket(const UdpFrame_t& msg, double timestamp)
 {
   pkt_pub_.publish(ToRosMsg(msg, timestamp));
 }
 
 inline void SourceDriver::SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg)
 {
   pub_.publish(ToRosMsg(msg, frame_id_));
 }
 
 inline void SourceDriver::SendCorrection(const u8Array_t& msg)
 {
   crt_pub_.publish(ToRosMsg(msg));
 }
 
 inline void SourceDriver::SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
 {
   loss_pub_.publish(ToRosMsg(total_packet_count, total_packet_loss_count));
 }
 
 inline void SourceDriver::SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
 {
   ptp_pub_.publish(ToRosMsg(ptp_lock_offset, ptp_status));
 }
 
 inline void SourceDriver::SendFiretime(const double *firetime_correction_)
 {
   firetime_pub_.publish(ToRosMsg(firetime_correction_));
 }
 
 inline void SourceDriver::SendImuConfig(const LidarImuData& msg)
 {
   imu_pub_.publish(ToRosMsg(msg));
 }
 
 inline sensor_msgs::PointCloud2 SourceDriver::ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id)
 {
   sensor_msgs::PointCloud2 ros_msg;
 
   int fields = 6;
   ros_msg.fields.clear();
   ros_msg.fields.reserve(fields);
   ros_msg.width = frame.points_num; 
   ros_msg.height = 1; 
 
   int offset = 0;
   offset = addPointField(ros_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
   offset = addPointField(ros_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
   offset = addPointField(ros_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
   offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
   offset = addPointField(ros_msg, "ring", 1, sensor_msgs::PointField::UINT16, offset);
   offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::PointField::FLOAT64, offset);
 
   ros_msg.point_step = offset;
   ros_msg.row_step = ros_msg.width * ros_msg.point_step;
   ros_msg.is_dense = false;
   ros_msg.data.resize(frame.points_num * ros_msg.point_step);
 
   sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
   sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
   sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
   sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
   sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
   sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
   for (size_t i = 0; i < frame.points_num; i++)
   {
     LidarPointXYZIRT point = frame.points[i];
     *iter_x_ = point.x;
     *iter_y_ = point.y;
     *iter_z_ = point.z;
     *iter_intensity_ = point.intensity;
     *iter_ring_ = point.ring;
     *iter_timestamp_ = point.timestamp;
     ++iter_x_;
     ++iter_y_;
     ++iter_z_;
     ++iter_intensity_;
     ++iter_ring_;
     ++iter_timestamp_;   
   }
   printf("frame:%d points:%u packet:%d start time:%lf end time:%lf\n",frame.frame_index, frame.points_num, frame.packet_num, frame.points[0].timestamp, frame.points[frame.points_num - 1].timestamp) ;
   // ros_msg.header.seq = s;
   int64_t sec = static_cast<int64_t>(frame.points[0].timestamp);  
   if (sec <= std::numeric_limits<int32_t>::max()) {
     ros_msg.header.stamp = ros::Time().fromSec(frame.points[0].timestamp);
   } else {
     printf("ros1 does not support timestamps greater than 19 January 2038 03:14:07 (now %lf)\n", frame.points[0].timestamp);
   }
   ros_msg.header.frame_id = frame_id_;
   return ros_msg;
 }
 
 inline hesai_ros_driver::UdpFrame SourceDriver::ToRosMsg(const UdpFrame_t& ros_msg, double timestamp) {
   hesai_ros_driver::UdpFrame rs_msg;
   for (size_t i = 0 ; i < ros_msg.size(); i++) {
     hesai_ros_driver::UdpPacket rawpacket;
     rawpacket.size = ros_msg[i].packet_len;
     rawpacket.data.resize(ros_msg[i].packet_len);
     memcpy(&rawpacket.data[0], &ros_msg[i].buffer[0], ros_msg[i].packet_len);
     rs_msg.packets.push_back(rawpacket);
   }
   int64_t sec = static_cast<int64_t>(timestamp);  
   if (sec <= std::numeric_limits<int32_t>::max()) {
     rs_msg.header.stamp = ros::Time().fromSec(timestamp);
   } else {
     printf("ros1 does not support timestamps greater than 19 January 2038 03:14:07 (now %lf)\n", timestamp);
   }
   rs_msg.header.frame_id = frame_id_;
   return rs_msg;
 }
 
 inline std_msgs::UInt8MultiArray SourceDriver::ToRosMsg(const u8Array_t& correction_string) {
   std_msgs::UInt8MultiArray msg;
   msg.data.resize(correction_string.size());
   std::copy(correction_string.begin(), correction_string.end(), msg.data.begin());
   return msg;
 }
 
 inline hesai_ros_driver::LossPacket SourceDriver::ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
 {
   hesai_ros_driver::LossPacket msg;
   msg.total_packet_count = total_packet_count;
   msg.total_packet_loss_count = total_packet_loss_count;  
   return msg;
 }
 
 inline hesai_ros_driver::Ptp SourceDriver::ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
 {
   hesai_ros_driver::Ptp msg;
   msg.ptp_lock_offset = ptp_lock_offset;
   std::copy(ptp_status.begin(), ptp_status.begin() + std::min(16ul, ptp_status.size()), msg.ptp_status.begin());
   return msg;
 }
 
 inline hesai_ros_driver::Firetime SourceDriver::ToRosMsg(const double *firetime_correction_)
 {
   hesai_ros_driver::Firetime msg;
   std::copy(firetime_correction_, firetime_correction_ + 512, msg.data.begin());
   return msg;
 }
 
 inline sensor_msgs::Imu SourceDriver::ToRosMsg(const LidarImuData &imu_config_)
 {
   sensor_msgs::Imu ros_msg;
   int64_t sec = static_cast<int64_t>(imu_config_.timestamp);  
   if (sec <= std::numeric_limits<int32_t>::max()) {
     ros_msg.header.stamp = ros::Time().fromSec(imu_config_.timestamp);
   } else {
     printf("ros1 does not support timestamps greater than 19 January 2038 03:14:07 (now %lf)\n", imu_config_.timestamp);
   }
   ros_msg.header.frame_id = frame_id_;
   ros_msg.linear_acceleration.x = imu_config_.imu_accel_x;
   ros_msg.linear_acceleration.y = imu_config_.imu_accel_y;
   ros_msg.linear_acceleration.z = imu_config_.imu_accel_z;
   ros_msg.angular_velocity.x = imu_config_.imu_ang_vel_x;
   ros_msg.angular_velocity.y = imu_config_.imu_ang_vel_y;
   ros_msg.angular_velocity.z = imu_config_.imu_ang_vel_z;
   return ros_msg;
 }
 
 inline void SourceDriver::RecievePacket(const hesai_ros_driver::UdpFrame& msg)
 {
   for (size_t i = 0; i < msg.packets.size(); i++) {
     driver_ptr_->lidar_ptr_->origin_packets_buffer_.emplace_back(&msg.packets[i].data[0], msg.packets[i].size);
   }
 }
 
 inline void SourceDriver::RecieveCorrection(const std_msgs::UInt8MultiArray& msg)
 {
   driver_ptr_->lidar_ptr_->correction_string_.resize(msg.data.size());
   std::copy(msg.data.begin(), msg.data.end(), driver_ptr_->lidar_ptr_->correction_string_.begin());
   while (1) {
     if (! driver_ptr_->lidar_ptr_->LoadCorrectionFromROSbag()) {
       break;
     }
   }
 }
 