#pragma once

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "reference-system-autoware/types.hpp"

namespace node {
struct SensorSettings {
  std::string node_name;
  std::string topic_name;
  std::chrono::nanoseconds cycle_time;
};

class Sensor : public rclcpp::Node {
 public:
  Sensor(const SensorSettings& settings) : Node(settings.node_name) {
    publisher_ = this->create_publisher<message_t>(settings.topic_name, 10);
    timer_ = this->create_wall_timer(settings.cycle_time,
                                     [this] { timer_callback(); });
  }

 private:
  void timer_callback() {
    auto message = publisher_->borrow_loaned_message();
    uint64_t timestamp =
        std::chrono::system_clock::now().time_since_epoch().count();
    message.get().data = std::to_string(timestamp);
    publisher_->publish(std::move(message));
  }

 private:
  publisher_t publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace node
