#include "joystick_controller.hpp"

double v_max_;
double w_max_;
bool CONTROL_MODE = false;

// JoystickEvent
bool JoystickEvent::is_axis() const {
  return (js.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS;
}

bool JoystickEvent::is_button() const {
  return (js.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON;
}

void JoystickEvent::parse(const struct js_event& e) {
  js = e;
  number = e.number;
  value = e.value;
}

// Joystick
Joystick::Joystick() {
  open_path("/dev/input/js0");
}

Joystick::Joystick(int joystickNumber) {
  std::stringstream ss;
  ss << "/dev/input/js" << joystickNumber;
  open_path(ss.str());
}

Joystick::Joystick(const std::string& devicePath) {
  device_path_ = devicePath;
  open_path(devicePath);
}

Joystick::~Joystick() {
  if (_fd >= 0) close(_fd);
}

bool Joystick::is_found() const {
  return _fd >= 0;
}

bool Joystick::sample(JoystickEvent* event) {
  struct js_event js;
  int bytes = read(_fd, &js, sizeof(struct js_event));

  if (bytes == sizeof(struct js_event)) {
    event->parse(js);
    return true;
  } else if (bytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return false;
  } else {
    if (_fd >= 0) {
      spdlog::warn("Joystick disconnected. Attempting to reconnect...");
      close(_fd);
      _fd = -1;
    }

    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      open_path(device_path_);
      if (_fd >= 0) {
        spdlog::info("Joystick reconnected successfully.");
        if (reconnect_callback_) reconnect_callback_(); 
        break;
      } else {
        spdlog::warn("Still waiting for joystick device...");
      }
    }

    return false;
  }
}

void Joystick::open_path(const std::string& path) {
  _fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
}

JoystickController::JoystickController(const std::string& devicePath, rclcpp::Node::SharedPtr node)
    : joystick(devicePath), node_(node) {

  joystick.set_reconnect_callback([this]() {
    this->reset_parameters_to_default();
  });

  if (!joystick.is_found()) {
    throw std::runtime_error("Failed to open joystick device.");
  }

  node_->declare_parameter("v_max", 0.5);
  node_->declare_parameter("w_max", 0.28);
  node_->get_parameter("v_max", v_max_);
  node_->get_parameter("w_max", w_max_);

  twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  spdlog::info("Joystick initialized and publisher created.");
}

void JoystickController::set_v_max(double new_val) {
  v_max_ = std::clamp(new_val, 0.0, 2.0);
  node_->set_parameter(rclcpp::Parameter("v_max", v_max_));
  spdlog::info("v_max updated: {:.2f}", v_max_);
}

void JoystickController::reset_parameters_to_default() {
  v_max_ = 0.5;
  w_max_ = 0.28;

  node_->set_parameter(rclcpp::Parameter("v_max", v_max_));
  node_->set_parameter(rclcpp::Parameter("w_max", w_max_));

  spdlog::info("Parameters reset to default. v_max = {:.2f}, w_max = {:.2f}", v_max_, w_max_);
}

void JoystickController::monitor_input() {
  spdlog::info("Start monitoring joystick...");
  while (rclcpp::ok()) {
    usleep(10000);  // 10ms = 100Hz

    JoystickEvent event;
    if (joystick.sample(&event)) {
      if (event.is_button()) {
        handle_button_event(event);
      }
      if (event.is_axis()) {
        handle_axis_event(event);
      }
    }

    if (buttonStates_[4] == 1) {
      CONTROL_MODE = true;
      publish_cmd_vel_from_axis();
    } else {
      CONTROL_MODE = false;
      geometry_msgs::msg::Twist stop_msg;
      twist_pub_->publish(stop_msg);
      // spdlog::info("Autonomous mode active.");
    }
  }
}

double JoystickController::apply_threshold(int value, double maxVelocity) {
  if (std::abs(value) < THRESHOLD) return 0.0;
  return value * (maxVelocity / 32767.0);
}

void JoystickController::publish_cmd_vel_from_axis() {
  double linear = apply_threshold(-axisValues_[1], v_max_);
  double angular = apply_threshold(-axisValues_[3], w_max_);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear;
  cmd.angular.z = angular;

  twist_pub_->publish(cmd);
  // spdlog::info("Published from axis: linear = {:.2f}, angular = {:.2f}", linear, angular);
}

void JoystickController::handle_axis_event(const JoystickEvent& event) {
  if (event.number == 1 || event.number == 3) {
    axisValues_[event.number] = event.value;

    double linear = apply_threshold(-axisValues_[1], v_max_);
    double angular = apply_threshold(-axisValues_[3], w_max_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;

    twist_pub_->publish(cmd);
    // spdlog::info("Published: linear = {:.2f}, angular = {:.2f}", linear, angular);
  }
}

void JoystickController::handle_button_event(const JoystickEvent& event) {
  buttonStates_[event.number] = event.value;

  std::string str;
  for (int i = 0; i < BUTTON_COUNT; ++i) {
    str += std::to_string(buttonStates_[i]);
    if (i < BUTTON_COUNT - 1) str += ", ";
  }

  if (event.number == 5 && event.value == 1) {
    set_v_max(v_max_ + 0.1);
  }

  if (event.number == 7 && event.value == 1) {
    set_v_max(v_max_ - 0.1);
  }
  // spdlog::info("Button states: [{}]", str);
}
