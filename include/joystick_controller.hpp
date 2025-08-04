#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <spdlog/spdlog.h>
#include <errno.h>
#include <algorithm>
#include <functional>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define THRESHOLD 8000
#define BUTTON_COUNT 12

extern double v_max_;
extern double w_max_;
extern bool CONTROL_MODE;

struct JoystickEvent {
  struct js_event js;
  bool is_axis() const;
  bool is_button() const;
  uint8_t number = 0;
  int16_t value = 0;

  void parse(const struct js_event& e);
};

class Joystick {
public:
  Joystick();
  Joystick(int joystickNumber);
  Joystick(const std::string& devicePath);
  ~Joystick();

  bool is_found() const;
  bool sample(JoystickEvent* event);
  void set_reconnect_callback(std::function<void()> cb) { reconnect_callback_ = cb; }

private:
  int _fd = -1;
  std::string device_path_;
  void open_path(const std::string& path);
  std::function<void()> reconnect_callback_;
};

class JoystickController {
public:
  JoystickController(const std::string& devicePath, rclcpp::Node::SharedPtr node);
  void monitor_input();
  void set_v_max(double new_val);
  void reset_parameters_to_default();
  void publish_cmd_vel_from_axis();

private:
  Joystick joystick;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  int axisValues_[8] = {0};
  int buttonStates_[BUTTON_COUNT] = {0};

  double apply_threshold(int value, double maxVelocity);
  void handle_axis_event(const JoystickEvent& event);
  void handle_button_event(const JoystickEvent& event);
};
