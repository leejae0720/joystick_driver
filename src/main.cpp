#include "joystick_controller.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joystick_cmd_vel_node");

  JoystickController* controller = nullptr;

  while (rclcpp::ok()) {
    try {
      controller = new JoystickController("/dev/input/js0", node);
      spdlog::info("Joystick device successfully connected.");
      break;
    } catch (const std::exception& e) {
      spdlog::warn("Joystick not found. Retrying in 1s... ({})", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  if (controller != nullptr) {
    controller->monitorInput();
    delete controller;
  }

  rclcpp::shutdown();
  return 0;
}
