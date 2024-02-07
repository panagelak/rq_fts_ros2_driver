#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/wrench_stamped.hpp>

class WrenchSimulator : public rclcpp::Node {
public:
  WrenchSimulator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("wrench_simulator", options) {}
  bool initialize(std::string topic_name) {
    sub_wrench_ =
        this->create_subscription<geometry_msgs::msg::WrenchStamped>(topic_name, 10, std::bind(&WrenchSimulator::wrenchCB, this, std::placeholders::_1));
    node_ = shared_from_this();
    sim_wrench_.header.stamp = shared_from_this()->get_clock()->now();
    sim_wrench_.header.frame_id = "";
    sim_wrench_.wrench.force.x = 0.0;
    sim_wrench_.wrench.force.y = 0.0;
    sim_wrench_.wrench.force.z = 0.0;
    sim_wrench_.wrench.torque.x = 0.0;
    sim_wrench_.wrench.torque.y = 0.0;
    sim_wrench_.wrench.torque.z = 0.0;
    thread_ = std::make_shared<std::thread>(&WrenchSimulator::spin, this);
    return true;
  }

  void spin() {
    executor_.add_node(node_);
    executor_.spin();
  }

  geometry_msgs::msg::WrenchStamped read_wrench() {
    return sim_wrench_;
  }

private:
  void wrenchCB(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr &msg) {
    sim_wrench_ = *msg.get();
  }

  // operator services
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<std::thread> thread_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_;
  geometry_msgs::msg::WrenchStamped sim_wrench_;
};
