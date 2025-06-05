#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <spdlog/spdlog.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define THRESHOLD 8000
#define BUTTON_COUNT 12

double v_max_;
double w_max_;

bool CONTROL_MODE = false; // false: autonomous, true: manual

struct JoystickEvent {
  struct js_event js;
  bool isAxis() const { return (js.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS; }
  bool isButton() const { return (js.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON; }
  uint8_t number = 0;
  int16_t value = 0;

  void parse(const struct js_event& e) {
    js = e;
    number = e.number;
    value = e.value;
  }
};

class Joystick {
public:
  Joystick() { openPath("/dev/input/js0"); }

  Joystick(int joystickNumber) {
    std::stringstream ss;
    ss << "/dev/input/js" << joystickNumber;
    openPath(ss.str());
  }

  Joystick(const std::string& devicePath) {
    openPath(devicePath);
  }

  ~Joystick() {
    if (_fd >= 0) close(_fd);
  }

  bool isFound() const { return _fd >= 0; }

  bool sample(JoystickEvent* event) {
    struct js_event js;
    int bytes = read(_fd, &js, sizeof(struct js_event));
    if (bytes == sizeof(struct js_event)) {
      event->parse(js);
      return true;
    }
    return false;
  }

private:
  int _fd = -1;

  void openPath(const std::string& path) {
      _fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
  }
};

class JoystickController {
public:
  JoystickController(const std::string& devicePath, rclcpp::Node::SharedPtr node)
      : joystick(devicePath), node_(node)
  {
    if (!joystick.isFound()) {
      throw std::runtime_error("Failed to open joystick device.");
    }

    // Declare parameters
    node_->declare_parameter("v_max", 0.5);
    node_->declare_parameter("w_max", 1.5);
    node_->get_parameter("v_max", v_max_);
    node_->get_parameter("w_max", w_max_);

    twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    spdlog::info("Joystick initialized and publisher created.");
  }

  void monitorInput() {
    spdlog::info("Start monitoring joystick...");
    while (rclcpp::ok()) {
      usleep(1000);
      JoystickEvent event;
      if (joystick.sample(&event)) {
        if (event.isButton()) {
          handleButtonEvent(event);
        }

        if (buttonStates_[5] == 1) {
          CONTROL_MODE = true;
          if (event.isAxis()) {
              handleAxisEvent(event);
          }
        } else {
          CONTROL_MODE = false;
          geometry_msgs::msg::Twist stop_msg;
          twist_pub_->publish(stop_msg);
          spdlog::info("Autonomous mode active.");
        }
      }
    }
  }

private:
  Joystick joystick;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  int axisValues_[8] = {0}; // 0:x, 1:y
  int buttonStates_[BUTTON_COUNT] = {0};

  double applyThreshold(int value, double maxVelocity) {
    if (std::abs(value) < THRESHOLD) return 0.0;
    return value * (maxVelocity / 32767.0);
  }

  void handleAxisEvent(const JoystickEvent& event) {
    if (event.number == 0 || event.number == 1) {
      axisValues_[event.number] = event.value;

      double linear = applyThreshold(-axisValues_[1], v_max_);
      double angular = applyThreshold(-axisValues_[0], w_max_);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = linear;
      cmd.angular.z = angular;

      twist_pub_->publish(cmd);
      spdlog::info("Published: linear = {:.2f}, angular = {:.2f}", linear, angular);
    }
  }

  void handleButtonEvent(const JoystickEvent& event) {
    buttonStates_[event.number] = event.value;

    std::string str;
    for (int i = 0; i < BUTTON_COUNT; ++i) {
      str += std::to_string(buttonStates_[i]);
      if (i < BUTTON_COUNT - 1) str += ", ";
    }
    spdlog::info("Button states: [{}]", str);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joystick_controller_node");

  try {
    JoystickController controller("/dev/input/js0", node);
    controller.monitorInput();
  } catch (const std::exception& e) {
    spdlog::error("Exception: {}", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
