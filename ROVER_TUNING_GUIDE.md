# Rover Kinematics Tuning Guide

## Parameters That Affect Motor RPM Calculation

All parameters are in `src/kinematics_node/config/rover_kinematics.yaml`

### 1. Physical Parameters (MEASURE THESE FIRST)

#### `wheel_base` (meters)
- **What it is**: Distance between left and right wheels
- **Current**: 0.150 m (15 cm)
- **How to measure**: Measure center-to-center distance between left and right wheel contact points
- **Effect on RPM**: 
  - Larger value = LESS RPM difference when turning (wider turns)
  - Smaller value = MORE RPM difference when turning (tighter turns)
- **Tuning**: If your rover turns too slowly, decrease this. If it spins too fast, increase it.

#### `wheel_diameter` (meters)
- **What it is**: Diameter of your wheels
- **Current**: 0.100 m (10 cm)
- **How to measure**: Measure wheel diameter with calipers or ruler
- **Effect on RPM**: 
  - Larger diameter = LOWER RPM for same speed (wheel covers more ground per rotation)
  - Smaller diameter = HIGHER RPM for same speed
- **Tuning**: CRITICAL - measure accurately! Wrong value means wrong speeds.

### 2. Speed Limits (SAFETY CONSTRAINTS)

#### `max_linear_speed` (m/s)
- **What it is**: Maximum forward/backward speed your rover can physically achieve
- **Current**: 0.8 m/s
- **Effect on RPM**: Sets the maximum possible RPM when moving straight
- **Tuning**: 
  - Test your rover at full throttle and measure actual speed
  - Set this to the measured maximum speed
  - Start conservative (0.5 m/s) and increase gradually

#### `max_angular_speed` (rad/s)
- **What it is**: Maximum rotation speed (how fast it can spin in place)
- **Current**: 2.0 rad/s (about 115°/second)
- **Effect on RPM**: Sets maximum RPM difference between left/right wheels when turning
- **Tuning**:
  - 1.0 rad/s = slow, stable turns
  - 2.0 rad/s = moderate turns
  - 3.0+ rad/s = fast, aggressive turns
  - Start low for stability

#### `max_rpm` (RPM)
- **What it is**: Hard limit on motor RPM (safety clamp)
- **Current**: 180.0 RPM
- **Effect on RPM**: Prevents any wheel from exceeding this RPM
- **Tuning**:
  - Check your motor specs for maximum safe RPM
  - Set 10-20% below motor maximum for safety margin
  - If motors stall under load, reduce this

### 3. Usage Factors (PERFORMANCE TUNING)

#### `linear_usage` (0.0 to 1.0)
- **What it is**: Percentage of max_linear_speed to actually use
- **Current**: 0.6 (60%)
- **Effect on RPM**: Scales down maximum forward/backward RPM
- **Formula**: `actual_max_speed = max_linear_speed × linear_usage`
- **Tuning**:
  - 0.3-0.5 = Conservative, good for indoor/testing
  - 0.6-0.7 = Moderate, balanced performance
  - 0.8-1.0 = Aggressive, full speed (use outdoors)
- **Why use this**: Allows quick speed adjustments without changing physical limits

#### `angular_usage` (0.0 to 1.0)
- **What it is**: Percentage of max_angular_speed to actually use
- **Current**: 0.5 (50%)
- **Effect on RPM**: Scales down maximum turning RPM difference
- **Formula**: `actual_max_turn = max_angular_speed × angular_usage`
- **Tuning**:
  - 0.3-0.4 = Gentle turns, very stable
  - 0.5-0.6 = Normal turns
  - 0.7-1.0 = Sharp turns, may be unstable
- **Why use this**: Prevents aggressive turning that could tip the rover

### 4. Control Parameters

#### `update_rate` (Hz)
- **What it is**: How often RPM commands are published
- **Current**: 20.0 Hz (50ms updates)
- **Effect on RPM**: Smoothness of control
- **Tuning**:
  - 10 Hz = Choppy but low CPU
  - 20-50 Hz = Smooth, recommended
  - 100+ Hz = Very smooth but higher CPU load

#### `cmd_vel_timeout` (seconds)
- **What it is**: How long to wait for new commands before stopping
- **Current**: 1.0 second
- **Effect on RPM**: Safety timeout - sets RPM to 0 if no commands received
- **Tuning**: Keep at 0.5-2.0 seconds for safety

---

## RPM Calculation Formula

```
For each wheel:

1. Get normalized input from teleop: u_lin, u_ang (range: -1 to +1)

2. Scale to real velocities:
   v = u_lin × linear_usage × max_linear_speed  (m/s)
   w = u_ang × angular_usage × max_angular_speed  (rad/s)

3. Differential drive kinematics:
   v_left  = v - (w × wheel_base / 2)
   v_right = v + (w × wheel_base / 2)

4. Convert to RPM:
   wheel_circumference = π × wheel_diameter
   rpm_left  = (v_left  / wheel_circumference) × 60
   rpm_right = (v_right / wheel_circumference) × 60

5. Clamp to limits:
   rpm_left  = clamp(rpm_left,  -max_rpm, +max_rpm)
   rpm_right = clamp(rpm_right, -max_rpm, +max_rpm)
```

---

## Tuning Workflow

### Step 1: Measure Physical Parameters
```bash
# Measure and update these FIRST:
wheel_diameter: [YOUR_MEASUREMENT]  # Use calipers!
wheel_base: [YOUR_MEASUREMENT]      # Center-to-center distance
```

### Step 2: Set Conservative Limits
```bash
max_linear_speed: 0.5    # Start slow
max_angular_speed: 1.0   # Start with gentle turns
max_rpm: 150.0           # Below motor max
linear_usage: 0.5        # 50% of max
angular_usage: 0.5       # 50% of max
```

### Step 3: Test and Tune
1. Run teleop and test forward motion
2. If too slow: increase `linear_usage` (0.6, 0.7, etc.)
3. If too fast/unstable: decrease `linear_usage`
4. Test turning
5. If turns too wide: decrease `wheel_base` slightly OR increase `angular_usage`
6. If turns too sharp: increase `wheel_base` slightly OR decrease `angular_usage`

### Step 4: Optimize for Your Use Case

**Indoor/Precision Work:**
```yaml
linear_usage: 0.4
angular_usage: 0.4
max_rpm: 120.0
```

**Outdoor/Speed:**
```yaml
linear_usage: 0.8
angular_usage: 0.6
max_rpm: 180.0
```

**Heavy Load/Torque:**
```yaml
linear_usage: 0.5
angular_usage: 0.4
max_rpm: 100.0  # Lower RPM = more torque
```

---

## Common Issues and Fixes

### Issue: Rover doesn't move straight
- **Cause**: Wheel diameter mismatch or motor calibration
- **Fix**: Verify wheel_diameter measurement, check if wheels are same size

### Issue: Turns are too wide/narrow
- **Cause**: Incorrect wheel_base
- **Fix**: Re-measure wheel_base, adjust by ±10mm and test

### Issue: Motors stall or overheat
- **Cause**: max_rpm too high or usage factors too high
- **Fix**: Reduce max_rpm by 20%, reduce usage factors to 0.5

### Issue: Rover is too slow
- **Cause**: Usage factors too low
- **Fix**: Increase linear_usage and angular_usage gradually

### Issue: Jerky motion
- **Cause**: update_rate too low
- **Fix**: Increase update_rate to 50 Hz

---

## Quick Reference: What to Change

| Want to...                    | Change this parameter      | Direction |
|-------------------------------|----------------------------|-----------|
| Go faster overall             | `linear_usage`             | Increase  |
| Turn faster                   | `angular_usage`            | Increase  |
| Reduce motor stress           | `max_rpm`                  | Decrease  |
| Make turns tighter            | `wheel_base`               | Decrease  |
| Make turns wider              | `wheel_base`               | Increase  |
| Smoother control              | `update_rate`              | Increase  |
| Fix incorrect speeds          | `wheel_diameter`           | Measure!  |
