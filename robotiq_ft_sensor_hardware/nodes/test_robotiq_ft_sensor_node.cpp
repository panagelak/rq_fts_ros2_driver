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
#include "robotiq_ft_sensor_hardware/rq_sensor_state.h"
#include "robotiq_ft_sensor_interfaces/msg/ft_sensor.hpp"
#include "robotiq_ft_sensor_interfaces/srv/sensor_accessor.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>

/*void receiveCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}*/

using namespace std::placeholders;
using namespace std::chrono_literals;

class RQTestSensor : public rclcpp::Node {
public:
  // Constructor
  RQTestSensor(const rclcpp::NodeOptions &options) : Node("rq_test_sensor", options) {}
  bool initialize() {
    sc_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sc_sensor_accessor_ =
        this->create_client<robotiq_ft_sensor_interfaces::srv::SensorAccessor>("robotiq_ft_sensor_acc", rmw_qos_profile_services_default, sc_cb_group_);

    rclcpp::SubscriptionOptions sub_options;
    sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = sub_cb_group_;
    sub_ft_sensor_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>("robotiq_force_torque_sensor_broadcaster/wrench", 10,
                                                                                  std::bind(&RQTestSensor::reCallback, this, _1), sub_options);
    timer_ = this->create_wall_timer(1ms, std::bind(&RQTestSensor::update, this));
    // update();
    return true;
  }

private:
  // ros
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<robotiq_ft_sensor_interfaces::srv::SensorAccessor>::SharedPtr sc_sensor_accessor_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_ft_sensor_;
  rclcpp::CallbackGroup::SharedPtr sc_cb_group_, sub_cb_group_;
  void reCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr &msg) {
    RCLCPP_INFO(rclcpp::get_logger("rq_test_sensor"), "I heard: FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f]", msg->wrench.force.x, msg->wrench.force.y,
                msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
  }
  int count = 0;
  bool update() {
    auto req = std::make_shared<robotiq_ft_sensor_interfaces::srv::SensorAccessor::Request>();
    if (count == 50) {
      req->command = "SET ZRO"; /// Deprecated Interface
      req->command_id = req->COMMAND_SET_ZERO;
      while (!sc_sensor_accessor_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
          return true;
        }
        RCLCPP_INFO(get_logger(), "service not available, waiting again...");
      }
      auto result = sc_sensor_accessor_->async_send_request(req, std::bind(&RQTestSensor::response_callback, this, std::placeholders::_1));
    }
    count += 1;
    return true;
  }
  bool service_done_ = false;
  void response_callback(rclcpp::Client<robotiq_ft_sensor_interfaces::srv::SensorAccessor>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      // uncomment below line if using Empty() message
      // RCLCPP_INFO(this->get_logger(), "Result: success");
      // comment below line if using Empty() message
      RCLCPP_INFO(get_logger(), "Result is [%s] and success [%d] ", future.get()->res.c_str(), future.get()->success);
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }
};

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<RQTestSensor>(node_options);
  node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
