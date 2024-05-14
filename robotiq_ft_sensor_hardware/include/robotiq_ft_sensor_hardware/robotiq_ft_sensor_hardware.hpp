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

#include <memory>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "robotiq_ft_sensor_interfaces/srv/sensor_accessor.hpp"
#include "rq_sensor_state.h"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace robotiq_ft_sensor_hardware
{
class RobotiqFTSensorHardware : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotiqFTSensorHardware);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  void read_background();

  // Store the sensor states for the simulated robot
  hardware_interface::ComponentInfo sensor_;
  std::array<double, 6> hw_sensor_states_ = { std::numeric_limits<double>::quiet_NaN() };
  realtime_tools::RealtimeBuffer<std::array<double, 6>> sensor_readings_;
  //=========== robotiq force torque
  rclcpp::Node::SharedPtr async_node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<std::thread> node_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_zero_fts_;
  rclcpp::Logger logger_{ rclcpp::get_logger("RobotiqFTSensorHardware") };
  // urdf parameters
  bool use_fake_mode_;
  int max_retries_ = 100;
  int read_rate_ = 10;
  std::string ftdi_id_;
  //
  INT_8 bufStream_[512];
  INT_8 ret_;
  void decode_message_and_do(INT_8 const* const buff, INT_8* const ret)
  {
    INT_8 get_or_set[3];
    INT_8 nom_var[4];

    if (buff == NULL || strlen(buff) != 7)
    {
      return;
    }

    strncpy(get_or_set, &buff[0], 3);
    strncpy(nom_var, &buff[4], strlen(buff) - 3);

    if (strstr(get_or_set, "GET"))
    {
      rq_state_get_command(nom_var, ret);
    }
    else if (strstr(get_or_set, "SET"))
    {
      if (strstr(nom_var, "ZRO"))
      {
        rq_state_do_zero_force_flag();
        strcpy(ret, "Done");
      }
    }
  }
  INT_8 sensor_state_machine()
  {
    if (ftdi_id_.empty())
    {
      return rq_sensor_state(max_retries_);
    }

    return rq_sensor_state(max_retries_, ftdi_id_);
  }
  void wait_for_other_connection()
  {
    INT_8 ret;

    while (rclcpp::ok())
    {
      RCLCPP_INFO(rclcpp::get_logger("RobotiqFTSensorHardware"), "Waiting for sensor connection...");
      usleep(1000000);  // Attend 1 seconde.

      ret = sensor_state_machine();
      RCLCPP_INFO(rclcpp::get_logger("RobotiqFTSensorHardware"), "ret is %d", ret);
      if (ret == 0)
      {
        RCLCPP_INFO(rclcpp::get_logger("RobotiqFTSensorHardware"), "Sensor connected!");
        return;
      }
    }
  }

  bool set_zero(std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    INT_8 buffer[512];
    std::string set_zero = "SET ZRO";
    decode_message_and_do((char*)set_zero.c_str(), buffer);

    return true;
  }

  /// New interface with numerical commands
  bool decode_message_and_do(robotiq_ft_sensor_interfaces::srv::SensorAccessor::Request::SharedPtr req,
                             robotiq_ft_sensor_interfaces::srv::SensorAccessor::Response::SharedPtr res)
  {
    INT_8 buffer[100];
    res->success = rq_state_get_command(req->command_id, buffer);
    res->res = buffer;

    if (!res->success)
    {
      RCLCPP_WARN(rclcpp::get_logger("RobotiqFTSensorHardware"),
                  "Unsupported command_id %i, should be in [%i, %i, %i, %i]", req->command_id,
                  req->COMMAND_GET_SERIAL_NUMBER, req->COMMAND_GET_FIRMWARE_VERSION, req->COMMAND_GET_PRODUCTION_YEAR,
                  req->COMMAND_SET_ZERO);
    }
    return res->success;
  }
};

}  // namespace robotiq_ft_sensor_hardware

#endif  // ROBOTIQ_FT_SENSOR__HARDWARE_INTERFACE_HPP_
