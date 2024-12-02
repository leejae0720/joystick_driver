# joystick++

A minimal C++ object-oriented API onto joystick devices under Linux.
This repository is based on [https://github.com/drewnoakes/joystick](https://github.com/drewnoakes/joystick)
It was developed to enable the control of mobile robots or other robots using a joystick.

## usage

Create an instance of `Joystick`:

```c++
Joystick joystick;
```
Setting your mobile robot or other robot linear velocity and angler velocity

```
#define V_MAX 0.8      // Maximum linear velocity (m/s)
#define W_MAX 3.11     // Maximum angular velocity (rad/s)
```

Ensure that it was found and that we can use it:

```c++
if (!joystick.isFound())
{
  printf("Failed to open joystick device.\n");
  return 1;
}
```

Sample events from the `Joystick`:

```c++
JoystickEvent event;
if (joystick.sample(&event))
{
  // use 'event'
}
```

## example

You might run this in a loop:

```c++
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
```

This produces something similar to:

    Axis: [Linear Velocity: 0.79 m/s, Angular Velocity: -1.22 rad/s]
    Buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

## options

You can specify the particular joystick by id:

```c++
Joystick js0(0);
Joystick js1(1);
```

Or provide a specific device name:

```c++
Joystick js0("/dev/input/js0");
```

## license

Released under [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).

## Contributor
 - Name: Jaehong Lee (이재홍)
 - Email: leejae0720@gmail.com
