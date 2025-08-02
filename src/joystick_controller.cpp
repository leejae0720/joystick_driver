#include "joystick_controller.hpp"

// 전역 변수 정의
double v_max_;
double w_max_;
bool CONTROL_MODE = false;

// JoystickEvent
bool JoystickEvent::isAxis() const {
  return (js.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS;
}
bool JoystickEvent::isButton() const {
  return (js.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON;
}
void JoystickEvent::parse(const struct js_event& e) {
  js = e;
  number = e.number;
  value = e.value;
}

// Joystick
Joystick::Joystick() {
  openPath("/dev/input/js0");
}
Joystick::Joystick(int joystickNumber) {
  std::stringstream ss;
  ss << "/dev/input/js" << joystickNumber;
  openPath(ss.str());
}
Joystick::Joystick(const std::string& devicePath) {
  openPath(devicePath);
}
Joystick::~Joystick() {
  if (_fd >= 0) close(_fd);
}
bool Joystick::isFound() const {
  return _fd >= 0;
}
bool Joystick::sample(JoystickEvent* event) {
  struct js_event js;
  int bytes = read(_fd, &js, sizeof(struct js_event));
  if (bytes == sizeof(struct js_event)) {
    event->parse(js);
    return true;
  }
  return false;
}
void Joystick::openPath(const std::string& path) {
  _fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
}

// JoystickController
JoystickController::JoystickController(const std::string& devicePath, rclcpp::Node::SharedPtr node)
    : joystick(devicePath), node_(node) {
  if (!joystick.isFound()) {
    throw std::runtime_error("Failed to open joystick device.");
  }

  node_->declare_parameter("v_max", 0.5);
  node_->declare_parameter("w_max", 1.5);
  node_->get_parameter("v_max", v_max_);
  node_->get_parameter("w_max", w_max_);

  twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  spdlog::info("Joystick initialized and publisher created.");
}

void JoystickController::monitorInput() {
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

double JoystickController::applyThreshold(int value, double maxVelocity) {
  if (std::abs(value) < THRESHOLD) return 0.0;
  return value * (maxVelocity / 32767.0);
}

void JoystickController::handleAxisEvent(const JoystickEvent& event) {
  if (event.number == 1 || event.number == 3) {
    axisValues_[event.number] = event.value;

    double linear = applyThreshold(-axisValues_[1], v_max_);
    double angular = applyThreshold(-axisValues_[3], w_max_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;

    twist_pub_->publish(cmd);
    spdlog::info("Published: linear = {:.2f}, angular = {:.2f}", linear, angular);
  }
}

void JoystickController::handleButtonEvent(const JoystickEvent& event) {
  buttonStates_[event.number] = event.value;

  std::string str;
  for (int i = 0; i < BUTTON_COUNT; ++i) {
    str += std::to_string(buttonStates_[i]);
    if (i < BUTTON_COUNT - 1) str += ", ";
  }
  spdlog::info("Button states: [{}]", str);
}
