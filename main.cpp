#include "joystick_controller.hpp"

int main()
{
    try
    {
        JoystickController controller("/dev/input/js0");
        controller.monitorInput();
    }
    catch (const std::exception &e)
    {
        spdlog::error("Error: Failed to open joystick device");
        return 1;
    }

    return 0;
}
