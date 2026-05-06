# roboteq_driver

A ROS2 driver for the Roboteq motor controller. Translates ROS2 topic commands into serial commands sent to the physical Roboteq hardware.


## Hardware Setup

The Roboteq controller is a physical board mounted on the robot. It receives serial commands over USB and drives current to the motors directly.

The computer running this node (e.g. an NVIDIA Jetson Orin) must be **physically connected** to the Roboteq board via USB, the driver opens `/dev/ttyACM0`, which only exists when the cable is plugged in. In a typical deployment the Orin is mounted on the robot itself, so the USB cable runs internally.

```
┌─────────────────────────────┐
│           Robot             │
│  ┌──────────┐  USB  ┌─────┐ │
│  │   Orin   │───────│ RBQ │ │
│  │ (ROS2)   │       │board│ │
│  └──────────┘       └──┬──┘ │
│                        │    │
│                   L motor  R motor
└─────────────────────────────┘
```

The Roboteq controller is ROS-agnostic, it only understands its own serial protocol. ROS is irrelevant to it.



## Architecture

The package is a single executable built from three layers:

```
driver_manager  →  Driver  →  RoboteqDevice  →  /dev/ttyACM0  →  Roboteq hardware
```

All three software layers run in the **same process on the same machine**. The only boundary that crosses physical hardware is the USB cable between `RoboteqDevice` and the Roboteq board.

### File Map

| File | Layer | Description |
|---|---|---|
| `src/driver_manager.cpp` | ROS2 | Node entry point. All ROS2 code lives here. |
| `include/roboteq_driver/driver.h` | C++ | `Driver` class. Wraps the Roboteq API into readable methods. |
| `include/roboteq_driver/RoboteqDevice.h/.cpp` | C++ | `RoboteqDevice` class. Handles raw serial communication. |
| `include/roboteq_driver/Constants.h` | C++ | Roboteq command/query/config constants (e.g. `_S`, `_BS`, `_DIN`). |
| `include/roboteq_driver/ErrorCodes.h` | C++ | Return codes (e.g. `RQ_SUCCESS`). |
| `include/Linux/RoboteqDevice.o` | Binary | Pre-compiled object file linked at build time. |

### Where ROS Lives

Only in `driver_manager.cpp`. Everything below it (`Driver`, `RoboteqDevice`, constants) is plain C++ with no ROS dependency.


## ROS2 Interface

### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `/motor_commands` | `robot_msgs/msg/MotorSpeedCommand` | Desired RPM for left and right wheels. Published by an upstream navigation or teleop node. |

### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/motor_rpms` | `robot_msgs/msg/MotorSpeedCommand` | Actual measured RPMs read back from the controller. Typically consumed by an odometry node. |
| `/joy_status` | `std_msgs/msg/Bool` | State of the green button (digital input pin 7). Used to enable/disable joystick control upstream. |



## Runtime Behaviour

The node runs at **100 Hz**. Each cycle it:

1. **Safety check** — if the safety button (pin 2) is pressed, both motors are commanded to 0 RPM regardless of `/motor_commands`.
2. **Motor commands** — otherwise, sends the latest left/right RPM values from `/motor_commands` to the controller.
3. **RPM feedback** — reads actual RPMs from the controller and publishes them to `/motor_rpms`.
4. **Button status** — reads the green button (pin 7) and publishes its state to `/joy_status`.

### Motor Channel Mapping

| Side | Roboteq Channel |
|---|---|
| Right | 1 |
| Left | 2 |

### Digital Input Pin Mapping

| Pin | Function |
|---|---|
| 2 | Safety button (stops motors when pressed) |
| 4 | Red button |
| 6 | Blue button |
| 7 | Green button (joystick enable) |


## Known Limitations

- **No command timeout** — if `/motor_commands` stops publishing, the last received command continues to be sent until the safety button is pressed.
- **Pre-compiled object file** — `RoboteqDevice.o` is linked as a binary, which may cause issues when cross-compiling for a different architecture.
- **Hardcoded serial port** — the device path `/dev/ttyACM0` is hardcoded in `driver.h`. If the USB port enumeration changes, the driver will fail to connect.
- **Specific Roboteq model unknown** — the code uses the generic Roboteq SDK which covers their entire product line. The exact controller model is not documented.
