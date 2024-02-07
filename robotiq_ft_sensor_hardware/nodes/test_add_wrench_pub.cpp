#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_wrench_publisher"), count_(0) {
    publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("add_fts_wrench", 10);
    timer_ = this->create_wall_timer(8ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = geometry_msgs::msg::WrenchStamped();
    message.header.frame_id = "";
    message.header.stamp = shared_from_this()->get_clock()->now();
    message.wrench.force.x = 1.0;
    message.wrench.force.y = 1.0;
    message.wrench.force.z = 1.0;
    message.wrench.torque.x = 1.0;
    message.wrench.torque.y = 1.0;
    message.wrench.torque.z = 1.0;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}