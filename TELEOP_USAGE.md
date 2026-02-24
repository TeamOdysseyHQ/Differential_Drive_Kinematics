# Rover Teleop and Monitoring Guide

## Build the Package

```bash
colcon build --packages-select kinematics_node
source install/setup.bash
```

## Running the System

You'll need 3 terminals:

### Terminal 1: Launch the Kinematics Node
```bash
source install/setup.bash
ros2 launch kinematics_node rover_kinematics.launch.py
```

This node:
- Subscribes to `/cmd_vel` (Twist messages)
- Publishes to `/teensy_motor_rpms` (Float32MultiArray with 6 wheel RPMs)

### Terminal 2: Run the Teleop Keyboard
```bash
source install/setup.bash
ros2 run kinematics_node teleop_keyboard
```

Controls:
- `w` - increase forward velocity
- `s` - decrease forward velocity (reverse)
- `a` - increase left turn (counter-clockwise)
- `d` - increase right turn (clockwise)
- `space` - emergency stop (zero all velocities)
- `q` - quit

### Terminal 3: Monitor RPM Output
```bash
source install/setup.bash
ros2 run kinematics_node monitor_rpms
```

This displays real-time RPM values for all 6 wheels:
- Wheels 0, 2, 4 = LEFT side
- Wheels 1, 3, 5 = RIGHT side

## Alternative: Use ROS2 CLI Tools

### Publish manually to cmd_vel:
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### Monitor RPM output:
```bash
ros2 topic echo /teensy_motor_rpms
```

## Troubleshooting

### Check active topics:
```bash
ros2 topic list
```

### Check topic info:
```bash
ros2 topic info /cmd_vel
ros2 topic info /teensy_motor_rpms
```

### Check node info:
```bash
ros2 node info /rover_kinematics_node
```
