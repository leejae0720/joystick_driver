#include "joystick.hh"
#include <unistd.h>
#include <cstdio>
#include <cmath>

#define V_MAX 0.8      // Maximum linear velocity (m/s)
#define W_MAX 3.11     // Maximum angular velocity (rad/s)
#define THRESHOLD 1500 // Noise threshold for axis values
#define BUTTON_COUNT 12 // Maximum number of buttons (adjust based on joystick)

double applyThreshold(int value, double maxVelocity) {
  if (std::abs(value) < THRESHOLD) {
    return 0.0; // Ignore small changes
  }
  return value * (maxVelocity / 32767.0); // Map to velocity
}

int main()
{
  Joystick joystick("/dev/input/js0");

  if (!joystick.isFound())
  {
    printf("Failed to open joystick device.\n");
    return 1;
  }

  printf("Joystick found. Monitoring input...\n");

  int axisValues[2] = {0, 0};                // Axis 1 (linear), Axis 0 (angular)
  int buttonStates[BUTTON_COUNT] = {0};     // Button states (pressed/released)

  while (true)
  {
    usleep(1000);

    JoystickEvent event;
    if (joystick.sample(&event))
    {
      if (event.isAxis() && (event.number == 0 || event.number == 1))
      {
        // Update axis values
        axisValues[event.number] = event.value;

        // Apply threshold filter and calculate velocities
        double linearVelocity = applyThreshold(-axisValues[1], V_MAX);
        double angularVelocity = applyThreshold(-axisValues[0], W_MAX);

        // Print axis velocities
        printf("Axis: [Linear Velocity: %.2f m/s, Angular Velocity: %.2f rad/s]\n", linearVelocity, angularVelocity);
      }
      else if (event.isButton())
      {
        // Update button state
        buttonStates[event.number] = event.value;

        // Print button states
        printf("Buttons: [");
        for (int i = 0; i < BUTTON_COUNT; ++i)
        {
          printf("%d", buttonStates[i]);
          if (i < BUTTON_COUNT - 1) printf(", ");
        }
        printf("]\n");
      }
    }
  }

  return 0;
}
