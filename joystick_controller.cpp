#include "joystick_controller.hpp"

bool CONTROL_MODE = false;  // define control mode (fasle: autonomous, true: manual)

// Joystick settings
Joystick::Joystick()
{
    openPath("/dev/input/js0");
}

Joystick::~Joystick()
{
    if (_fd >= 0)
        close(_fd);
}

Joystick::Joystick(int joystickNumber)
{
    std::stringstream sstm;
    sstm << "/dev/input/js" << joystickNumber;
    openPath(sstm.str());
}

Joystick::Joystick(std::string devicePath)
{
    openPath(devicePath);
}

Joystick::Joystick(std::string devicePath, bool blocking)
{
    openPath(devicePath, blocking);
}

bool Joystick::isFound()
{
    return _fd >= 0;
}

bool Joystick::sample(JoystickEvent *event)
{
    int bytes = read(_fd, event, sizeof(*event));
    return bytes == sizeof(*event);
}

// Joystick controller system implementation
JoystickController::JoystickController(const std::string &devicePath) : joystick(devicePath)
{
    if (!joystick.isFound())
    {
        throw std::runtime_error("Failed to open joystick device.");
    }
    spdlog::info("Joystick initialized successfully!");
}

/**
 * Apply threshold to axis value and map to velocity range.
 */
double JoystickController::applyThreshold(int value, double maxVelocity)
{
    if (std::abs(value) < THRESHOLD)
    {
        return 0.0; // Ignore small changes
    }
    return value * (maxVelocity / 32767.0); // Map to velocity
}

/**
 * Handle axis events.
 */
void JoystickController::handleAxisEvent(const JoystickEvent &event)
{
    if (event.number == 0 || event.number == 1)
    {
        axisValues[event.number] = event.value;

        double linearVelocity = applyThreshold(-axisValues[1], V_MAX);
        double angularVelocity = applyThreshold(-axisValues[0], W_MAX);

        spdlog::info("Axis: [Linear Velocity: {:.2f} m/s, Angular Velocity: {:.2f} rad/s]", linearVelocity, angularVelocity);
    }
}

/**
 * Handle button events.
 */
void JoystickController::handleButtonEvent(const JoystickEvent &event)
{
    buttonStates[event.number] = event.value;

    std::string buttonStatesStr;
    for (int i = 0; i < BUTTON_COUNT; ++i)
    {
        buttonStatesStr += std::to_string(buttonStates[i]);
        if (i < BUTTON_COUNT - 1)
        {
            buttonStatesStr += ", ";
        }
    }
    spdlog::info("Buttons: [{}]", buttonStatesStr);
}

/**
 * Main loop to monitor joystick input.
 */
void JoystickController::monitorInput()
{
    spdlog::info("Monitoring joystick input...");
    while (true)
    {
        usleep(1000);

        JoystickEvent event;
        if (joystick.sample(&event))
        {
            if (event.isButton())
            {
                handleButtonEvent(event);
            }

            // Check if the 6th button (index 5) is pressed
            if (buttonStates[5] == 1) // 6th button is active
            {
                CONTROL_MODE = true;
                if (event.isAxis())
                {
                    handleAxisEvent(event);
                }
            }
            else
            {
                CONTROL_MODE = false;
                spdlog::info("Autonomous mode control");
            }
        }
    }
}
