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
        fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }

    return 0;
}
