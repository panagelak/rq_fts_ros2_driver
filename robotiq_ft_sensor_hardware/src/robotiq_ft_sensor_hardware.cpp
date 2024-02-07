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

#include "robotiq_ft_sensor_hardware/robotiq_ft_sensor_hardware.hpp"

namespace robotiq_ft_sensor_hardware {
hardware_interface::CallbackReturn RobotiqFTSensorHardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  use_fake_mode_ = info_.hardware_parameters["use_fake_mode"] == "True" || info_.hardware_parameters["use_fake_mode"] == "true";
  use_add_fts_wrench_ = info_.hardware_parameters["use_add_fts_wrench"] == "True" || info_.hardware_parameters["use_add_fts_wrench"] == "true";
  if (use_add_fts_wrench_) {
    add_fts_wrench_topic_ = info_.hardware_parameters["add_fts_wrench_topic"];
  }
  hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  // parameters
  max_retries_ = std::stoi(info_.hardware_parameters["max_retries"]);
  ftdi_id_ = info_.hardware_parameters["ftdi_id"];
  if (ftdi_id_.size() == 1)
    ftdi_id_ = "";
  RCLCPP_INFO(logger_, "Parameter : use_fake_mode -> %d", use_fake_mode_);
  RCLCPP_INFO(logger_, "Parameter : use_add_fts_wrench -> %d", use_add_fts_wrench_);
  RCLCPP_INFO(logger_, "Parameter : add_fts_wrench_topic -> %s", add_fts_wrench_topic_.c_str());
  RCLCPP_INFO(logger_, "Parameter : max_retries -> %d", max_retries_);
  RCLCPP_INFO(logger_, "Parameter : ftdi_id -> %s", ftdi_id_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotiqFTSensorHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(logger_, "Exporting State Interfaces");
  for (auto &sensor : info_.sensors) {
    for (uint j = 0; j < sensor.state_interfaces.size(); ++j) {
      RCLCPP_INFO(logger_, "Sensor %s state %s", sensor.name.c_str(), sensor.state_interfaces[j].name.c_str());
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[j].name, &hw_sensor_states_[j]));
    }
  }
  return state_interfaces;
}

hardware_interface::CallbackReturn RobotiqFTSensorHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating ...please wait...");
  if (!use_fake_mode_) {
    // Connect
    if (!ftdi_id_.empty()) {
      RCLCPP_INFO(logger_, "Trying to connect to a sensor at /dev/%s", ftdi_id_.c_str());
    } else {
      RCLCPP_INFO(logger_, "No device filename specified. Will attempt to discover Robotiq force torque sensor.");
    }
    // Connect
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
  }
  //=== ASYNC NODE
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=robotiq_ft_hardware_internal_" + info_.name});
  async_node_ = rclcpp::Node::make_shared("_", options);
  srv_sensor_accessor_ = async_node_->create_service<robotiq_ft_sensor_interfaces::srv::SensorAccessor>(
      "robotiq_ft_sensor_acc", std::bind(&RobotiqFTSensorHardware::receiverCallback, this, std::placeholders::_1, std::placeholders::_2));
  if (use_add_fts_wrench_) {
    sub_add_wrench_ = async_node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
        add_fts_wrench_topic_, 10, std::bind(&RobotiqFTSensorHardware::wrenchAddCB, this, std::placeholders::_1));
  }
  node_thread_ = std::make_unique<std::thread>([&]() {
    executor_.add_node(async_node_);
    executor_.spin();
    executor_.remove_node(async_node_);
  });

  RCLCPP_INFO(logger_, "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotiqFTSensorHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(logger_, "Deactivating ...please wait...");
  //
  executor_.cancel();
  node_thread_->join();
  node_thread_.reset();
  srv_sensor_accessor_.reset();
  sub_add_wrench_.reset();
  async_node_.reset();
  //
  // TODO DEACTIVE RQ SENSOR
  //
  RCLCPP_INFO(logger_, "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotiqFTSensorHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (use_fake_mode_) {
    hw_sensor_states_[0] = 0.0;
    hw_sensor_states_[1] = 0.0;
    hw_sensor_states_[2] = 0.0;
    hw_sensor_states_[3] = 0.0;
    hw_sensor_states_[4] = 0.0;
    hw_sensor_states_[5] = 0.0;
  } else {
    ret_ = sensor_state_machine();
    if (ret_ == -1) {
      wait_for_other_connection();
    }

    if (rq_sensor_get_current_state() == RQ_STATE_RUN) {
      strcpy(bufStream_, "");
      // auto msgStream = get_data();

      if (rq_state_got_new_message()) {
        hw_sensor_states_[0] = rq_state_get_received_data(0);
        hw_sensor_states_[1] = rq_state_get_received_data(1);
        hw_sensor_states_[2] = rq_state_get_received_data(2);
        hw_sensor_states_[3] = rq_state_get_received_data(3);
        hw_sensor_states_[4] = rq_state_get_received_data(4);
        hw_sensor_states_[5] = rq_state_get_received_data(5);
      }
    }
  }
  if (use_add_fts_wrench_) {
    hw_sensor_states_[0] += add_wrench_msg_.wrench.force.x;
    hw_sensor_states_[1] += add_wrench_msg_.wrench.force.y;
    hw_sensor_states_[2] += add_wrench_msg_.wrench.force.z;
    hw_sensor_states_[3] += add_wrench_msg_.wrench.torque.x;
    hw_sensor_states_[4] += add_wrench_msg_.wrench.torque.y;
    hw_sensor_states_[5] += add_wrench_msg_.wrench.torque.z;
  }
  return hardware_interface::return_type::OK;
}

} // namespace robotiq_ft_sensor_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robotiq_ft_sensor_hardware::RobotiqFTSensorHardware, hardware_interface::SensorInterface)
