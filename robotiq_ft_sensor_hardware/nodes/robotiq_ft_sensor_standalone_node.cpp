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

#include <stdio.h>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <exception>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "robotiq_ft_sensor_interfaces/msg/ft_sensor.hpp"
#include "robotiq_ft_sensor_interfaces/srv/sensor_accessor.hpp"
#include "std_msgs/msg/string.hpp"
#include <robotiq_ft_sensor_hardware/rq_sensor_state.h>
#include <sstream>

using namespace std::placeholders;
using namespace std::chrono_literals;

class RQSensor : public rclcpp::Node {
public:
  // Constructor
  RQSensor(const rclcpp::NodeOptions &options) : Node("rq_sensor", options) {}
  bool initialize() {
    // parameters
    max_retries_ = get_parameter_or<int>("max_retries", 100);
    ftdi_id = get_parameter_or<std::string>("ftdi_id", "");
    wrenchMsg_.header.frame_id = get_parameter_or<std::string>("frame_id", "robotiq_ft_frame_id");
    // Connect to Sensor
    if (!ftdi_id.empty()) {
      RCLCPP_INFO(get_logger(), "Trying to connect to a sensor at /dev/%s", ftdi_id.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "No device filename specified. Will attempt to discover Robotiq force torque sensor.");
    }

    // If we can't initialize, we return an error
    ret_ = sensor_state_machine();
    if (ret_ == -1) {
      wait_for_other_connection();
    }
    // Reads basic info on the sensor
    ret_ = sensor_state_machine();
    if (ret_ == -1) {
      wait_for_other_connection();
    }
    // Starts the stream
    ret_ = sensor_state_machine();
    if (ret_ == -1) {
      wait_for_other_connection();
    }
    // zero the sensor
    // set_zero();

    //
    srv_sensor_accessor_ = this->create_service<robotiq_ft_sensor_interfaces::srv::SensorAccessor>(
        "robotiq_ft_sensor_acc", std::bind(&RQSensor::receiverCallback, this, std::placeholders::_1, std::placeholders::_2));
    // pub_sensor_ = this->create_publisher<robotiq_ft_sensor_interfaces::msg::FTSensor>("robotiq_ft_sensor", 512);
    pub_wrench_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("robotiq_force_torque_sensor_broadcaster/wrench", 512);
    timer_ = this->create_wall_timer(8ms, std::bind(&RQSensor::update, this));
    RCLCPP_INFO(get_logger(), "Starting Sensor!!!!!!");
    return true;
  }
  void set_zero() {
    INT_8 buffer[512];
    std::string set_zero = "SET ZRO";
    decode_message_and_do((char *)set_zero.c_str(), buffer);
  }
  bool receiverCallback(std::shared_ptr<robotiq_ft_sensor_interfaces::srv::SensorAccessor::Request> req,
                        std::shared_ptr<robotiq_ft_sensor_interfaces::srv::SensorAccessor::Response> res) {

    /// Support for old string-based interface
    if (req->command.length()) {
      RCLCPP_WARN_ONCE(get_logger(), "Usage of command-string is deprecated, please use the numeric command_id");
      RCLCPP_INFO(get_logger(), "I heard: [%s]", req->command.c_str());
      INT_8 buffer[512];
      decode_message_and_do((char *)req->command.c_str(), buffer);
      res->res = buffer;
      RCLCPP_INFO(get_logger(), "I send: [%s]", res->res.c_str());
      return true;
    }

    /// New interface with numerical commands
    decode_message_and_do(req, res);
    return true;
  }
  bool update() {
    ret_ = sensor_state_machine();
    if (ret_ == -1) {
      wait_for_other_connection();
    }

    if (rq_sensor_get_current_state() == RQ_STATE_RUN) {
      strcpy(bufStream_, "");
      msgStream_ = get_data();

      if (rq_state_got_new_message()) {
        // pub_sensor_->publish(msgStream_);

        // compose WrenchStamped Msg
        wrenchMsg_.header.stamp = shared_from_this()->get_clock()->now();
        wrenchMsg_.wrench.force.x = msgStream_.fx;
        wrenchMsg_.wrench.force.y = msgStream_.fy;
        wrenchMsg_.wrench.force.z = msgStream_.fz;
        wrenchMsg_.wrench.torque.x = msgStream_.mx;
        wrenchMsg_.wrench.torque.y = msgStream_.my;
        wrenchMsg_.wrench.torque.z = msgStream_.mz;
        pub_wrench_->publish(wrenchMsg_);
      }
    }
    return true;
  }

private:
  // params
  int max_retries_ = 100;
  std::string ftdi_id;
  geometry_msgs::msg::WrenchStamped wrenchMsg_;
  robotiq_ft_sensor_interfaces::msg::FTSensor msgStream_;
  INT_8 bufStream_[512];
  INT_8 ret_;
  // ros
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<robotiq_ft_sensor_interfaces::srv::SensorAccessor>::SharedPtr srv_sensor_accessor_;
  rclcpp::Publisher<robotiq_ft_sensor_interfaces::msg::FTSensor>::SharedPtr pub_sensor_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_;

  //
  bool decode_message_and_do(robotiq_ft_sensor_interfaces::srv::SensorAccessor::Request::SharedPtr req,
                             robotiq_ft_sensor_interfaces::srv::SensorAccessor::Response::SharedPtr res) {
    INT_8 buffer[100];
    res->success = rq_state_get_command(req->command_id, buffer);
    res->res = buffer;

    if (!res->success) {
      RCLCPP_WARN(get_logger(), "Unsupported command_id %i, should be in [%i, %i, %i, %i]", req->command_id, req->COMMAND_GET_SERIAL_NUMBER,
                  req->COMMAND_GET_FIRMWARE_VERSION, req->COMMAND_GET_PRODUCTION_YEAR, req->COMMAND_SET_ZERO);
    }

    return res->success;
  }
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
    if (ftdi_id.empty()) {
      return rq_sensor_state(max_retries_);
    }

    return rq_sensor_state(max_retries_, ftdi_id);
  }
  void wait_for_other_connection() {
    INT_8 ret;

    while (rclcpp::ok()) {
      RCLCPP_INFO(get_logger(), "Waiting for sensor connection...");
      usleep(1000000); // Attend 1 seconde.

      ret = sensor_state_machine();
      RCLCPP_INFO(get_logger(), "ret is %d", ret);
      if (ret == 0) {
        RCLCPP_INFO(get_logger(), "Sensor connected!");
        return;
      }
      // ros::spinOnce();
      // executor_.spin_once();
    }
  }
  robotiq_ft_sensor_interfaces::msg::FTSensor get_data(void) {
    robotiq_ft_sensor_interfaces::msg::FTSensor msgStream;

    msgStream.fx = rq_state_get_received_data(0);
    msgStream.fy = rq_state_get_received_data(1);
    msgStream.fz = rq_state_get_received_data(2);
    msgStream.mx = rq_state_get_received_data(3);
    msgStream.my = rq_state_get_received_data(4);
    msgStream.mz = rq_state_get_received_data(5);

    return msgStream;
  }
};

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<RQSensor>(node_options);
  node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
