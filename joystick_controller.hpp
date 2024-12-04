#ifndef __JOYSTICK_CONTROLLER_HPP__
#define __JOYSTICK_CONTROLLER_HPP__

#include <cmath>
#include <cstdio>
#include <unistd.h>
#include <stdexcept>
#include <iostream>
#include <string>
#include <fcntl.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <spdlog/spdlog.h>

// Constants for joystick event types
#define JS_EVENT_BUTTON 0x01 // button pressed/released
#define JS_EVENT_AXIS   0x02 // joystick moved
#define JS_EVENT_INIT   0x80 // initial state of device

/**
 * Encapsulates all data relevant to a sampled joystick event.
 */
class JoystickEvent
{
public:
    static const short MIN_AXES_VALUE = -32768;
    static const short MAX_AXES_VALUE = 32767;

    unsigned int time;
    short value;
    unsigned char type;
    unsigned char number;

    bool isButton() { return (type & JS_EVENT_BUTTON) != 0; }
    bool isAxis() { return (type & JS_EVENT_AXIS) != 0; }
    bool isInitialState() { return (type & JS_EVENT_INIT) != 0; }

    friend std::ostream &operator<<(std::ostream &os, const JoystickEvent &e)
    {
        os << "type=" << static_cast<int>(e.type)
           << " number=" << static_cast<int>(e.number)
           << " value=" << static_cast<int>(e.value);
        return os;
    }
};

/**
 * Represents a joystick device. Allows data to be sampled from it.
 */
class Joystick
{

public:
~Joystick();
Joystick();
Joystick(int joystickNumber);
Joystick(std::string devicePath);
Joystick(std::string devicePath, bool blocking);

bool isFound();

bool sample(JoystickEvent *event);

private:
    int _fd;

    void openPath(std::string devicePath, bool blocking = false)
    {
        _fd = open(devicePath.c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);
    }
};

/**
 * Controller class to process joystick events and handle inputs.
 */
class JoystickController
{

public:
    explicit JoystickController(const std::string &devicePath = "/dev/input/js0");

    void monitorInput();
    
private:
    static constexpr double V_MAX = 0.8;       // Maximum linear velocity (m/s)
    static constexpr double W_MAX = 3.11;      // Maximum angular velocity (rad/s)
    static constexpr int THRESHOLD = 1500;     // Noise threshold for axis values
    static constexpr int BUTTON_COUNT = 12;    // Maximum number of button

    Joystick joystick;
    int axisValues[2] = {0, 0};                // Axis 1 (linear), Axis 0 (angular)
    int buttonStates[BUTTON_COUNT] = {0};      // Button states (pressed/released)

    double applyThreshold(int value, double maxVelocity);

    void handleAxisEvent(const JoystickEvent &event);

    void handleButtonEvent(const JoystickEvent &event);
};

#endif // __JOYSTICK_CONTROLLER_HPP__
