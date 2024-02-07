/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Robotiq, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of Robotiq, Inc. nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright (c) 2014, Robotiq, Inc
 */

#ifndef ROBOTIQ_FT_SENSOR__HARDWARE_INTERFACE_HPP_
#define ROBOTIQ_FT_SENSOR__HARDWARE_INTERFACE_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
// #include "robotiq_ft_sensor_interfaces/msg/ft_sensor.hpp"
#include "robotiq_ft_sensor_interfaces/srv/sensor_accessor.hpp"
#include "rq_sensor_state.h"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace robotiq_ft_sensor_hardware {
class RobotiqFTSensorHardware : public hardware_interface::SensorInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotiqFTSensorHardware);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  void wrenchAddCB(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr &msg) {
    add_wrench_msg_ = *msg.get();
  }
  // Parameters for the RRBot simulation

  // Store the sensor states for the simulated robot
  std::vector<double> hw_sensor_states_;
  //=========== robotiq force torque
  rclcpp::Node::SharedPtr async_node_;
  std::unique_ptr<std::thread> node_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Service<robotiq_ft_sensor_interfaces::srv::SensorAccessor>::SharedPtr srv_sensor_accessor_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_add_wrench_;
  geometry_msgs::msg::WrenchStamped add_wrench_msg_;
  rclcpp::Logger logger_{rclcpp::get_logger("RobotiqFTSensorHardware")};
  // urdf parameters
  bool use_fake_mode_;
  bool use_add_fts_wrench_;
  std::string add_fts_wrench_topic_;
  int max_retries_ = 100;
  std::string ftdi_id_;
  //
  INT_8 bufStream_[512];
  INT_8 ret_;
  void decode_message_and_do(INT_8 const *const buff, INT_8 *const ret) {
    INT_8 get_or_set[3];
    INT_8 nom_var[4];

    if (buff == NULL || strlen(buff) != 7) {
      return;
    }

    strncpy(get_or_set, &buff[0], 3);
    strncpy(nom_var, &buff[4], strlen(buff) - 3);

    if (strstr(get_or_set, "GET")) {
      rq_state_get_command(nom_var, ret);
    } else if (strstr(get_or_set, "SET")) {
      if (strstr(nom_var, "ZRO")) {
        rq_state_do_zero_force_flag();
        strcpy(ret, "Done");
      }
    }
  }
  INT_8 sensor_state_machine() {
    if (ftdi_id_.empty()) {
      return rq_sensor_state(max_retries_);
    }

    return rq_sensor_state(max_retries_, ftdi_id_);
  }
  void wait_for_other_connection() {
    INT_8 ret;

    while (rclcpp::ok()) {
      RCLCPP_INFO(rclcpp::get_logger("RobotiqFTSensorHardware"), "Waiting for sensor connection...");
      usleep(1000000); // Attend 1 seconde.

      ret = sensor_state_machine();
      RCLCPP_INFO(rclcpp::get_logger("RobotiqFTSensorHardware"), "ret is %d", ret);
      if (ret == 0) {
        RCLCPP_INFO(rclcpp::get_logger("RobotiqFTSensorHardware"), "Sensor connected!");
        return;
      }
    }
  }
  void set_zero() {
    INT_8 buffer[512];
    std::string set_zero = "SET ZRO";
    decode_message_and_do((char *)set_zero.c_str(), buffer);
  }
  // robotiq_ft_sensor_interfaces::msg::FTSensor get_data(void) {
  //   robotiq_ft_sensor_interfaces::msg::FTSensor msgStream;

  //   msgStream.fx = rq_state_get_received_data(0);
  //   msgStream.fy = rq_state_get_received_data(1);
  //   msgStream.fz = rq_state_get_received_data(2);
  //   msgStream.mx = rq_state_get_received_data(3);
  //   msgStream.my = rq_state_get_received_data(4);
  //   msgStream.mz = rq_state_get_received_data(5);

  //   return msgStream;
  // }
  bool receiverCallback(std::shared_ptr<robotiq_ft_sensor_interfaces::srv::SensorAccessor::Request> req,
                        std::shared_ptr<robotiq_ft_sensor_interfaces::srv::SensorAccessor::Response> res) {
    /// Support for old string-based interface
    if (req->command.length()) {
      RCLCPP_WARN_ONCE(rclcpp::get_logger("RobotiqFTSensorHardware"), "Usage of command-string is deprecated, please use the numeric command_id");
      RCLCPP_INFO(rclcpp::get_logger("RobotiqFTSensorHardware"), "I heard: [%s]", req->command.c_str());
      INT_8 buffer[512];
      decode_message_and_do((char *)req->command.c_str(), buffer);
      res->res = buffer;
      RCLCPP_INFO(rclcpp::get_logger("RobotiqFTSensorHardware"), "I send: [%s]", res->res.c_str());
      return true;
    }

    /// New interface with numerical commands
    decode_message_and_do(req, res);
    return true;
  }
  /// New interface with numerical commands
  bool decode_message_and_do(robotiq_ft_sensor_interfaces::srv::SensorAccessor::Request::SharedPtr req,
                             robotiq_ft_sensor_interfaces::srv::SensorAccessor::Response::SharedPtr res) {
    INT_8 buffer[100];
    res->success = rq_state_get_command(req->command_id, buffer);
    res->res = buffer;

    if (!res->success) {
      RCLCPP_WARN(rclcpp::get_logger("RobotiqFTSensorHardware"), "Unsupported command_id %i, should be in [%i, %i, %i, %i]", req->command_id,
                  req->COMMAND_GET_SERIAL_NUMBER, req->COMMAND_GET_FIRMWARE_VERSION, req->COMMAND_GET_PRODUCTION_YEAR, req->COMMAND_SET_ZERO);
    }
    return res->success;
  }
};

} // namespace robotiq_ft_sensor_hardware

#endif // ROBOTIQ_FT_SENSOR__HARDWARE_INTERFACE_HPP_
