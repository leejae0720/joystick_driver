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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define THRESHOLD 8000
#define BUTTON_COUNT 12

extern double v_max_;
extern double w_max_;
extern bool CONTROL_MODE;

struct JoystickEvent {
  struct js_event js;
  bool isAxis() const;
  bool isButton() const;
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

  bool isFound() const;
  bool sample(JoystickEvent* event);

private:
  int _fd = -1;
  std::string device_path_;
  void openPath(const std::string& path);
};

class JoystickController {
public:
  JoystickController(const std::string& devicePath, rclcpp::Node::SharedPtr node);
  void monitorInput();

private:
  Joystick joystick;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  int axisValues_[8] = {0};
  int buttonStates_[BUTTON_COUNT] = {0};

  double applyThreshold(int value, double maxVelocity);
  void handleAxisEvent(const JoystickEvent& event);
  void handleButtonEvent(const JoystickEvent& event);
};
