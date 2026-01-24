# Rover Kinematics Node

ROS2 package for 6-wheel differential drive rover kinematics.

## Overview

This node subscribes to `/cmd_vel` (geometry_msgs/Twist) and converts linear.x and angular.z velocities into individual wheel RPMs for a 6-wheel differential drive rover. The RPMs are published as a Float32MultiArray to `/teensy_motor_rpms`.

## Package Structure

```
src/kinematics_node/
├── kinematics_node/
│   ├── __init__.py
│   └── rover_kinematics_node.py
├── config/
│   └── rover_kinematics.yaml
├── launch/
│   └── rover_kinematics.launch.py
├── resource/
│   └── kinematics_node
├── package.xml
├── setup.py
└── setup.cfg
```

## Building

```bash
cd <workspace_root>
colcon build --packages-select kinematics_node
source install/setup.bash
```

## Running

```bash
# With default parameters
ros2 run kinematics_node rover_kinematics_node

# With launch file (loads config)
ros2 launch kinematics_node rover_kinematics.launch.py
```

## Parameters

- `wheel_base`: Distance between left and right wheels (m)
- `wheel_diameter`: Wheel diameter (m)
- `max_linear_speed`: Maximum linear velocity (m/s)
- `max_angular_speed`: Maximum angular velocity (rad/s)
- `linear_usage`: Fraction of max linear speed to use (0-1)
- `angular_usage`: Fraction of max angular speed to use (0-1)
- `max_rpm`: Maximum RPM clamp
- `cmd_vel_timeout`: Safety timeout for cmd_vel (s)
- `input_cmd_topic`: Input topic name (default: `/cmd_vel`)
- `output_topic`: Output topic name (default: `/teensy_motor_rpms`)
- `update_rate`: Publishing rate (Hz)

## Topics

### Subscribed
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands

### Published
- `/teensy_motor_rpms` (std_msgs/Float32MultiArray): 6 wheel RPMs [left0, right1, left2, right3, left4, right5]

## Wheel Mapping

- Wheels 0, 2, 4: Left side
- Wheels 1, 3, 5: Right side
