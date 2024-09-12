/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file waypoint_v2.hpp
 *
 * @brief Header file for the WaypointV2Module class
 *
 * @authors Umesh Mane
 * Contact: umeshmane280@gmail.com
 *
 */

#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WAYPOINT_V2_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_WAYPOINT_V2_HPP_

#include "dji_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_waypoint_v2.h"
// #include "dji_waypoint_v2_type.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <pugixml.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <shared_mutex>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "math.h"

typedef struct
{
  uint8_t eventID;
  char* eventStr;
} waypoint_v2_event_str;

typedef struct
{
  uint8_t missionState;
  char* stateStr;
} waypoint_v2_state_str;

namespace psdk_ros2
{
class WaypointV2Module : public rclcpp_lifecycle::LifecycleNode
{
 public:
  /**
   * @brief Construct a new WaypointV2Module object
   * @param node_name Name of the node
   */
  explicit WaypointV2Module(const std::string& name);

  /**
   * @brief Destroy the WaypointV2Module object
   */
  ~WaypointV2Module();

  /**
   * @brief Configures the WaypointV2Module. Creates the ROS 2 subscribers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief Activates the WaypointV2Module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Cleans the WaypointV2Module. Resets the ROS 2 subscribers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state);
  /**
   * @brief Deactivates the WaypointV2Module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Shuts down the WaypointV2Module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief Initialize the WaypointV2Module.
   * @return true/false
   */
  bool init();

  /**
   * @brief Deinitialize the WaypointV2Module
   * @return true/false
   */
  bool deinit();

  // static const waypoint_v2_event_str s_waypoint_v2_event_str[];

  // static const waypoint_v2_state_str s_waypoint_v2_state_str[];

  //   struct PerceptionParams
  //   {
  //     std::string perception_camera_frame;
  //   };
  //   PerceptionParams params_;

 private:
  void start_v2_waypoint_mission(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  friend T_DjiReturnCode c_waypoint_v2_event_callback(
      T_DjiWaypointV2MissionEventPush eventData);
  friend uint8_t c_waypoint_v2_get_mission_event_index(uint8_t eventID);
  friend T_DjiReturnCode c_waypoint_v2_state_callback(
      T_DjiWaypointV2MissionStatePush stateData);

  void extract_kmz(const std::string& kmz_file, const std::string& output_dir);

  std::string parse_kmz(const std::string& output_dir);

  T_DjiReturnCode waypoint_v2_event_callback(
      T_DjiWaypointV2MissionEventPush eventData);

  uint8_t waypoint_v2_get_mission_event_index(uint8_t eventID);

  T_DjiReturnCode waypoint_v2_state_callback(
      T_DjiWaypointV2MissionStatePush stateData);

  uint8_t waypoint_v2_get_mission_state_index(uint8_t state);

  void waypoint_v2_upload_mission(const std::string parse_kmz_file);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_v2_waypoint_mission_;

  //   /* Streaming callbacks */
  //   /**
  //    * @brief Stereo camera stream of both left and right camera sensor
  //    */
  //   void perception_image_callback(T_DjiPerceptionImageInfo imageInfo,
  //                                uint8_t* imageRawBuffer, uint32_t
  //                                bufferLen);
  //   /**
  //    * @brief Stereo camera parameters publisher
  //    * publish camera parametes of selected direction
  //    * @return true/false True if pervious camera strem cleared successfully
  //    * otherwise false
  //    */
  //   void perception_camera_parameters_publisher();
  //   /**
  //    * @brief Start Perception Streaming
  //    * @param request Perception stereo camera stream Direction
  //    * DOWN = 0, FRONT = 1, REAR = 2, UP = 3, LEFT = 4, RIGHT = 5
  //    * @param response PerceptionStereoVisionSetup service response
  //    */
  //   void start_perception_cb(
  //       const std::shared_ptr<PerceptionStereoVisionSetup::Request> request,
  //       const std::shared_ptr<PerceptionStereoVisionSetup::Response>
  //       response);

  //   /**
  //    * @brief Start the perception stereo camera stream
  //    * @param stereo_cameras_direction select perception stereo cameras
  //    direction
  //    * @return true/false Returns true if the streaming has been started
  //    * correctly and False otherwise.
  //    */
  //   bool start_perception_stereo_cameras_stream(
  //       const uint stereo_cameras_direction);

  //   /**
  //    * @brief Stop the perception stereo camera stream
  //    * @param stereo_cameras_direction select perception stereo cameras
  //    direction
  //    * @return true/false Returns true if the streaming has been stoped
  //    * correctly and False otherwise.
  //    */
  //   bool stop_perception_stereo_cameras_stream(
  //       const uint stereo_cameras_direction);

  //   /**
  //   @brief Clear the previous perception stereo camera stream
  //   */
  //   bool clear_perception_stereo_cameras_stream();

  //   rclcpp::Service<PerceptionStereoVisionSetup>::SharedPtr
  //       perception_stereo_vision_service_;
  //   rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
  //       perception_stereo_vision_left_pub_;
  //   rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
  //       perception_stereo_vision_right_pub_;
  //   rclcpp_lifecycle::LifecyclePublisher<
  //       psdk_interfaces::msg::PerceptionCameraParameters>::SharedPtr
  //       perception_camera_parameters_pub_;

  //   // Timer for publishing camera parameters at 20 hz
  //   rclcpp::TimerBase::SharedPtr timer_;

  bool is_module_initialized_{false};
  //   int stereo_cameras_direction_;
  //   /**
  //    * Populate the direction map for perception stereo camera direction.
  //    * refer typedef enum E_DjiPerceptionDirection for more information.
  //    */
  //   std::unordered_map<std::string, uint8_t> direction_map_ = {
  //       {"DOWN", 0}, {"FRONT", 1}, {"REAR", 2},
  //       {"UP", 3},   {"LEFT", 4},  {"RIGHT", 5}};

  //   std::vector<E_DjiPerceptionDirection> perception_image_direction = {
  //       DJI_PERCEPTION_RECTIFY_DOWN, DJI_PERCEPTION_RECTIFY_FRONT,
  //       DJI_PERCEPTION_RECTIFY_REAR, DJI_PERCEPTION_RECTIFY_UP,
  //       DJI_PERCEPTION_RECTIFY_LEFT, DJI_PERCEPTION_RECTIFY_RIGHT};
  static const waypoint_v2_event_str s_waypoint_v2_event_str[];

  static const waypoint_v2_state_str s_waypoint_v2_state_str[];
  mutable std::shared_mutex global_ptr_mutex_;
};

extern std::shared_ptr<WaypointV2Module> global_waypoint_v2_ptr_;

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_PERCEPTION_HPP_
