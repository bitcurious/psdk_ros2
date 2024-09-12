/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file waypoint_v2.cpp
 *
 * @brief Waypoint_v2 module implementation. This module is responsible for
 * handling the waypoint_v2 mission on drone.
 *
 * @authors Umesh Mane
 * Contact: umeshmane280@gmail.com
 *
 */

#include "psdk_wrapper/modules/waypoint_v2.hpp"

namespace psdk_ros2
{
const waypoint_v2_event_str WaypointV2Module::s_waypoint_v2_event_str[] = {
    {.eventID = 0x01, .eventStr = "Interrupt Event"},
    {.eventID = 0x02, .eventStr = "Resume Event"},
    {.eventID = 0x03, .eventStr = "Stop Event"},
    {.eventID = 0x10, .eventStr = "Arrival Event"},
    {.eventID = 0x11, .eventStr = "Finished Event"},
    {.eventID = 0x12, .eventStr = "Avoid Obstacle Event"},
    {.eventID = 0x30, .eventStr = "Action Switch Event"},
    {.eventID = 0xFF, .eventStr = "Unknown"}};

const waypoint_v2_state_str WaypointV2Module::s_waypoint_v2_state_str[] = {
    {.missionState = 0x00, .stateStr = "Ground station not start"},
    {.missionState = 0x01, .stateStr = "Mission prepared"},
    {.missionState = 0x02, .stateStr = "Enter mission"},
    {.missionState = 0x03, .stateStr = "Execute mission"},
    {.missionState = 0x04, .stateStr = "Pause Mission"},
    {.missionState = 0x05, .stateStr = "Enter mission after ending pause"},
    {.missionState = 0x06, .stateStr = "Exit mission"},
    {.missionState = 0xFF, .stateStr = "Unknown"}};

WaypointV2Module::WaypointV2Module(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))

{
  RCLCPP_INFO(get_logger(), "Creating PerceptionModule");
}

WaypointV2Module::~WaypointV2Module()
{
  RCLCPP_INFO(get_logger(), "Destroying PerceptionModule");
}

WaypointV2Module::CallbackReturn
WaypointV2Module::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring WaypointV2Module");
  start_v2_waypoint_mission_ = create_service<std_srvs::srv::Trigger>(
      "/wrapper/psdk_ros2/start_v2_waypoint_mission",
      std::bind(&WaypointV2Module::start_v2_waypoint_mission, this,
                std::placeholders::_1, std::placeholders::_2));
  //   perception_stereo_vision_left_pub_ =
  //       create_publisher<sensor_msgs::msg::Image>(
  //           "psdk_ros2/perception_stereo_left_stream",
  //           rclcpp::SensorDataQoS());
  return CallbackReturn::SUCCESS;
}

WaypointV2Module::CallbackReturn
WaypointV2Module::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating WaypointV2Module");
  //   perception_camera_parameters_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

WaypointV2Module::CallbackReturn
WaypointV2Module::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating WaypointV2Module");
  //   perception_camera_parameters_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

WaypointV2Module::CallbackReturn
WaypointV2Module::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up WaypointV2Module");
  start_v2_waypoint_mission_.reset();
  //   perception_camera_parameters_pub_.reset();
  return CallbackReturn::SUCCESS;
}

WaypointV2Module::CallbackReturn
WaypointV2Module::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutting down WaypointV2Module");
  std::unique_lock<std::shared_mutex> lock(global_ptr_mutex_);
  global_waypoint_v2_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

bool
WaypointV2Module::init()
{
  if (is_module_initialized_)
  {
    RCLCPP_WARN(get_logger(),
                "Waypoint v2 module is already initialized, skipping.");
    return true;
  }
  RCLCPP_INFO(get_logger(), "Initiating Waypoint_v2 module");
  T_DjiReturnCode return_code = DjiWaypointV2_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not initialize waypoint_v2 module. Error code: %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = true;
  return true;
}

bool
WaypointV2Module::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing waypoint_v2 module");
  T_DjiReturnCode return_code = DjiWaypointV2_Deinit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not deinitialize the waypoint_v2 module. Error code: %ld",
        return_code);
    return false;
  }
  is_module_initialized_ = false;
  return true;
}

T_DjiReturnCode
c_waypoint_v2_event_callback(T_DjiWaypointV2MissionEventPush eventData)
{
  std::unique_lock<std::shared_mutex> lock(
      global_waypoint_v2_ptr_->global_ptr_mutex_);
  return global_waypoint_v2_ptr_->waypoint_v2_event_callback(eventData);
}

uint8_t
c_waypoint_v2_get_mission_event_index(uint8_t eventID)
{
  std::unique_lock<std::shared_mutex> lock(
      global_waypoint_v2_ptr_->global_ptr_mutex_);
  return global_waypoint_v2_ptr_->waypoint_v2_get_mission_event_index(eventID);
}

T_DjiReturnCode
c_waypoint_v2_state_callback(T_DjiWaypointV2MissionStatePush stateData)
{
  std::unique_lock<std::shared_mutex> lock(
      global_waypoint_v2_ptr_->global_ptr_mutex_);
  return global_waypoint_v2_ptr_->waypoint_v2_state_callback(stateData);
}
void
WaypointV2Module::start_v2_waypoint_mission(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Waypoint mission starting");
  std::string kmz_file =
      "/home/ubuntu/psdk_ws/src/dji_rc_controller/waypoint_kmz/"
      "waypoint_v3_test_file.kmz";
  std::string output_dir =
      "/home/ubuntu/psdk_ws/src/dji_rc_controller/waypoint_kmz";
  extract_kmz(kmz_file, output_dir);
  std::string parse_kmz_file = parse_kmz(output_dir);
  T_DjiReturnCode return_code;
  RCLCPP_INFO(get_logger(), "Register Mission event callback");
  return_code =
      DjiWaypointV2_RegisterMissionEventCallback(&c_waypoint_v2_event_callback);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Register waypoint V2 event failed, error code: 0x%08X",
                 return_code);
  }
  // osalHandler->TaskSleepMs(timeOutMs);
  RCLCPP_INFO(get_logger(), "Register Mission state callback");
  return_code =
      DjiWaypointV2_RegisterMissionStateCallback(&c_waypoint_v2_state_callback);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Register waypoint V2 state failed, error code: 0x%08X",
                 return_code);
  }
  RCLCPP_INFO(get_logger(), "Uploding mission");
  waypoint_v2_upload_mission(parse_kmz_file);
  // osalHandler->TaskSleepMs(timeOutMs);
}

void
WaypointV2Module::extract_kmz(const std::string &kmz_file,
                              const std::string &output_dir)
{
  std::string command = "unzip -o " + kmz_file + " -d " + output_dir;
  int result = std::system(command.c_str());
  if (result != 0)
  {
    RCLCPP_ERROR(get_logger(), "Failed to extract KMZ file.");
    exit(1);
  }
}

std::string
WaypointV2Module::parse_kmz(const std::string &output_dir)
{
  std::string kml_file = output_dir + "/wpmz/waylines.wpml";
  std::ifstream file(kml_file);
  if (!file.is_open())
  {
    RCLCPP_ERROR(get_logger(), "Failed to open KML file: %s", kml_file.c_str());
  }
  return kml_file;
}

T_DjiReturnCode
WaypointV2Module::waypoint_v2_event_callback(
    T_DjiWaypointV2MissionEventPush eventData)

{
  if (eventData.event == 0x01)
  {
    RCLCPP_INFO(get_logger(), "[%s]: Mission interrupted reason is 0x%x",
                s_waypoint_v2_event_str[c_waypoint_v2_get_mission_event_index(
                                            eventData.event)]
                    .eventStr,
                eventData.data.interruptReason);
  }
  else if (eventData.event == 0x02)
  {
    RCLCPP_INFO(get_logger(), "[%s]: Mission recover reason is 0x%x",
                s_waypoint_v2_event_str[c_waypoint_v2_get_mission_event_index(
                                            eventData.event)]
                    .eventStr,
                eventData.data.recoverProcess);
  }
  else if (eventData.event == 0x03)
  {
    RCLCPP_INFO(get_logger(), "[%s]: Mission exit reason is 0x%x",
                s_waypoint_v2_event_str[c_waypoint_v2_get_mission_event_index(
                                            eventData.event)]
                    .eventStr,
                eventData.data.exitReason);
  }
  else if (eventData.event == 0x10)
  {
    RCLCPP_INFO(get_logger(), "[%s]: Current waypoint index is %d",
                s_waypoint_v2_event_str[c_waypoint_v2_get_mission_event_index(
                                            eventData.event)]
                    .eventStr,
                eventData.data.waypointIndex);
  }
  else if (eventData.event == 0x11)
  {
    RCLCPP_INFO(
        get_logger(), "[%s]: Current mission execute times is %d",
        s_waypoint_v2_event_str[c_waypoint_v2_get_mission_event_index(
                                    eventData.event)]
            .eventStr,
        eventData.data.T_DjiWaypointV2MissionExecEvent.currentMissionExecTimes);
  }
  else if (eventData.event == 0x12)
  {
    RCLCPP_INFO(get_logger(), "[%s]: avoid obstacle state:%d",
                s_waypoint_v2_event_str[c_waypoint_v2_get_mission_event_index(
                                            eventData.event)]
                    .eventStr,
                eventData.data.avoidState);
  }
  else if (eventData.event == 0x30)
  {
    RCLCPP_INFO(get_logger(),
                "[%s]: action id:%d, pre actuator state:%d, current actuator "
                "state:%d, result:0x%08llX",
                s_waypoint_v2_event_str[c_waypoint_v2_get_mission_event_index(
                                            eventData.event)]
                    .eventStr,
                eventData.data.T_DjiWaypointV2ActionExecEvent.actionId,
                eventData.data.T_DjiWaypointV2ActionExecEvent.preActuatorState,
                eventData.data.T_DjiWaypointV2ActionExecEvent.curActuatorState,
                eventData.data.T_DjiWaypointV2ActionExecEvent.result);
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

uint8_t
WaypointV2Module::waypoint_v2_get_mission_event_index(uint8_t eventID)
{
  uint8_t i;
  for (i = 0;
       i < sizeof(s_waypoint_v2_event_str) / sizeof(waypoint_v2_event_str); i++)
  {
    if (s_waypoint_v2_event_str[i].eventID == eventID)
    {
      return i;
    }
  }
  return i;
}

T_DjiReturnCode
WaypointV2Module::waypoint_v2_state_callback(
    T_DjiWaypointV2MissionStatePush stateData)
{
  static uint32_t curMs = 0;
  static uint32_t preMs = 0;
  rclcpp::Time curTime = this->get_clock()->now();
  curMs = curTime.seconds() * 1000;
  // &curMs = this->get_clock()->now();
  if (curMs - preMs >= 1000)
  {
    preMs = curMs;
    RCLCPP_INFO(get_logger(),
                "[Waypoint Index:%d]: State: %s, velocity:%.2f m/s",
                stateData.curWaypointIndex,
                s_waypoint_v2_state_str[waypoint_v2_get_mission_state_index(
                                            stateData.state)]
                    .stateStr,
                (dji_f32_t)stateData.velocity / 100);
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

uint8_t
WaypointV2Module::waypoint_v2_get_mission_state_index(uint8_t state)
{
  uint8_t i;
  for (i = 0;
       i < sizeof(s_waypoint_v2_state_str) / sizeof(waypoint_v2_state_str); i++)
  {
    if (s_waypoint_v2_state_str[i].missionState == state)
    {
      return i;
    }
  }
  return i;
}

void
WaypointV2Module::waypoint_v2_upload_mission(const std::string parse_kmz_file)
{
  std::ifstream file(parse_kmz_file);
  RCLCPP_INFO(get_logger(), "Upload mission fn called");
  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string kml_content = buffer.str();

  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_string(kml_content.c_str());
  if (!result)
  {
    RCLCPP_ERROR(get_logger(), "Failed to parse KML file.");
    return;
  }
  T_DjiReturnCode returnCode;
  T_DjiWayPointV2MissionSettings missionInitSettings = {0};
  T_DJIWaypointV2ActionList actionList = {NULL, 0};
  missionInitSettings.repeatTimes = 1;
  pugi::xml_node missionConfig =
      doc.child("kml").child("Document").child("wpml:missionConfig");
  if (!missionConfig)
  {
    RCLCPP_ERROR(get_logger(), "Missing missionConfig element in KML file.");
    return;
  }

  auto flyToWaylineMode =
      missionConfig.child("wpml:flyToWaylineMode").text().as_string();
  if (std::string(flyToWaylineMode) == "safely")
  {
    missionInitSettings.gotoFirstWaypointMode =
        DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;
  }
  else if (std::string(flyToWaylineMode) == "pointToPoint")
  {
    missionInitSettings.gotoFirstWaypointMode =
        DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_POINT_TO_POINT;
  }

  auto finishAction =
      missionConfig.child("wpml:finishAction").text().as_string();
  if (std::string(finishAction) == "goHome")
  {
    missionInitSettings.finishedAction = DJI_WAYPOINT_V2_FINISHED_GO_HOME;
  }
  else if (std::string(finishAction) == "noAction")
  {
    missionInitSettings.finishedAction = DJI_WAYPOINT_V2_FINISHED_NO_ACTION;
  }
  else if (std::string(finishAction) == "autoLand")
  {
    missionInitSettings.finishedAction = DJI_WAYPOINT_V2_FINISHED_AUTO_LANDING;
  }
  else if (std::string(finishAction) == "gotoFirstWaypoint")
  {
    missionInitSettings.finishedAction =
        DJI_WAYPOINT_V2_FINISHED_GO_TO_FIRST_WAYPOINT;
  }

  auto exitOnRCLost =
      missionConfig.child("wpml:exitOnRCLost").text().as_string();
  if (std::string(exitOnRCLost) == "goContinue")
  {
    missionInitSettings.actionWhenRcLost =
        DJI_WAYPOINT_V2_MISSION_STOP_WAYPOINT_V2_AND_EXECUTE_RC_LOST_ACTION;
  }
  else if (std::string(exitOnRCLost) == "executeLostAction")
  {
    missionInitSettings.actionWhenRcLost =
        DJI_WAYPOINT_V2_MISSION_KEEP_EXECUTE_WAYPOINT_V2;
  }

  auto executeRCLostAction =
      missionConfig.child("wpml:executeRCLostAction").text().as_string();
  int takeOffSecurityHeight =
      missionConfig.child("wpml:takeOffSecurityHeight").text().as_int();

  int globalTransitionalSpeed =
      missionConfig.child("wpml:globalTransitionalSpeed").text().as_int();
  if (globalTransitionalSpeed)
  {
    missionInitSettings.maxFlightSpeed = globalTransitionalSpeed;
  }
  else
  {
    missionInitSettings.maxFlightSpeed = 10;
  }

  missionInitSettings.autoFlightSpeed = 2;

  // Extract all <wpml:index> values
  std::vector<int> indexValues;
  for (pugi::xml_node placemark :
       doc.child("kml").child("Document").child("Folder").children("Placemark"))
  {
    int index = placemark.child("wpml:index").text().as_int();
    indexValues.push_back(index);
  }
  int missionNum = *std::max_element(indexValues.begin(), indexValues.end());

  RCLCPP_INFO(get_logger(), "Maximum index value: %d ", missionNum);
  missionInitSettings.missTotalLen = missionNum + 2;
}

}  // namespace psdk_ros2
