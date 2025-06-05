int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("joystick_cmd_vel_node");

    try
    {
        JoystickController controller("/dev/input/js0", node);
        controller.monitorInput();
    }
    catch (const std::exception &e)
    {
        spdlog::error("Exception: {}", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
