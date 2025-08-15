Looking at the wall_following_final_version.py code, I can explain why there's always a rotational speed when the robot is going forward. Let me examine the control logic: Now let me check the control parameters to see the distance thresholds: Let me find the control parameters section: ## Explanation: Why There's Always Rotational Speed (To the Right)

Based on my analysis of the code, here's exactly what's happening:

### 🎯 **The Control Logic Uses a "Bang-Bang" Controller:**

The robot's control system works in **three distinct zones**:

1. **Too Far From Wall** (`right_dist > 0.3m`): 
   ```python
   twist.angular.z = -self.turn_speed    # -0.3 rad/s = turn RIGHT
   ```

2. **Too Close To Wall** (`right_dist < 0.2m`):
   ```python
   twist.angular.z = self.turn_speed     # +0.3 rad/s = turn LEFT
   ```

3. **Optimal Range** (`0.2m ≤ right_dist ≤ 0.3m`):
   ```python
   twist.angular.z = 0.0                 # No turning
   ```

### 🔄 **Why You See Constant Right Turning:**

The constant rightward rotation you're observing is **NOT coded together with linear speed** - it's because:

1. **Narrow Optimal Zone**: The "go straight" zone is only **0.1 meters wide** (0.2m to 0.3m)
2. **Real-World Noise**: In practice, the laser readings fluctuate slightly due to:
   - Sensor noise
   - Robot vibration while moving
   - Measurement precision limits
   - Environmental factors

3. **Bang-Bang Behavior**: This type of controller constantly "hunts" around the target distance, causing:
   - `right_dist = 0.31m` → Turn RIGHT (angular.z = -0.3)
   - `right_dist = 0.29m` → Go STRAIGHT (angular.z = 0.0)  
   - `right_dist = 0.31m` → Turn RIGHT again...

### 📊 **The Root Cause:**

If you're seeing **persistent** rightward turning, it likely means:

```
right_dist > 0.3m (consistently)
```

This indicates the robot is consistently **too far from the wall**, so the control system is trying to get closer by turning right.

### 🛠️ **Solutions to Reduce Oscillation:**

1. **Widen the Optimal Zone:**
   ```python
   self.min_wall_distance = 0.15   # Wider deadband
   self.max_wall_distance = 0.35   # 0.2m wide instead of 0.1m
   ```

2. **Add Proportional Control (PID):**
   ```python
   # Instead of bang-bang, use proportional control
   error = right_dist - self.target_wall_distance
   twist.angular.z = -self.kp * error  # Smoother response
   ```

3. **Add Hysteresis:**
   ```python
   # Different thresholds for entering/exiting turn states
   if self._was_turning_right:
       if right_dist < 0.25:  # Lower threshold to exit right turn
           twist.angular.z = 0.0
   else:
       if right_dist > 0.35:  # Higher threshold to start right turn
           twist.angular.z = -self.turn_speed
   ```

The current implementation is a simple but effective bang-bang controller that will naturally oscillate around the target distance. The constant right turning indicates the robot is generally too far from the wall and is trying to correct this by moving closer.
