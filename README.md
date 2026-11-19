# Moorebot Scout — ROS 2 Humble Fallback Race Controller (`moorebot_race`)

Open-loop ROS 2 Humble fallback controller for navigating a Moorebot Scout through a rectangular U-shaped race track from either the **left** or **right** entrance.

The node automatically discovers `/<robot>/cmd_vel` from `mqtt_ros2_bridge.py`[cite: 3] and streams `geometry_msgs/msg/Twist` commands at `20 Hz` to stay ahead of the bridge's `0.5 s` deadman timeout[cite: 3].

---

## 1. Install & Build

```bash
# Clone into your ROS 2 Humble workspace
mkdir -p ~/moorebot_ws/src
cd ~/moorebot_ws/src
git clone [https://github.com/](https://github.com/)<your-username>/moorebot_race.git

# Build with symlinks (Python edits apply immediately without rebuilding)
cd ~/moorebot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 2. Race-Day Runbook (Copy & Paste)

### Step 1 — Connect to the Moorebot Hotspot
1. Power on the Moorebot and wait ~1 minute to boot[cite: 4]. Hold the right-side button if needed to enter hotspot mode (phone icon)[cite: 4].
2. Connect your laptop Wi-Fi to `robot-scout-...` (`10.42.0.1`)[cite: 4]:
   * **`swarmmetal`:** `r0123456`[cite: 4]
   * **All other Moorebots:** `scout0123456`[cite: 4]

### Step 2 — Start the ROS 2 Bridge (Terminal 1)
Keep this running in the background for the entire session[cite: 4].

```bash
cd ~/moorebot_humble_all_in_one
bash start_bridge.sh
```

### Step 3 — Verify Robot Discovery (Terminal 2)
Confirm `/<robot>/cmd_vel` is active before launching[cite: 3, 4]:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

### Step 4 — Launch the Fallback Race Script (Terminal 2)
Press **`Ctrl+C`** at any time to immediately stop the robot.

**Start from LEFT entrance** *(Straight → Right → Straight → Right → Straight)*:
```bash
source /opt/ros/humble/setup.bash
source ~/moorebot_ws/install/setup.bash

ros2 launch moorebot_race race_fallback.launch.py \
  start_side:=left \
  long_distance:=1.5 \
  short_distance:=1.0 \
  linear_speed:=0.25 \
  angular_speed:=0.8 \
  drift_angular_trim:=-0.05 \
  distance_scale:=1.0 \
  turn_scale:=1.5 \
  pause_duration:=0.4 \
  start_delay:=2.0
```

**Start from RIGHT entrance** *(Straight → Left → Straight → Left → Straight)*:
```bash
source /opt/ros/humble/setup.bash
source ~/moorebot_ws/install/setup.bash

ros2 launch moorebot_race race_fallback.launch.py \
  start_side:=right \
  long_distance:=1.5 \
  short_distance:=1.0 \
  linear_speed:=0.25 \
  angular_speed:=0.8 \
  drift_angular_trim:=-0.05 \
  distance_scale:=1.0 \
  turn_scale:=1.5 \
  pause_duration:=0.4 \
  start_delay:=2.0
```

---

## 3. Configurable Parameters

All launch arguments can be passed as `param:=value` to `ros2 launch moorebot_race race_fallback.launch.py`.

| Parameter | Default | Type | Description & Tuning Notes |
| :--- | :--- | :--- | :--- |
| `start_side` | `'left'` | string | Entrance side: `'left'` (turns right, `-angular.z`) or `'right'` (turns left, `+angular.z`). |
| `robot_name` | `''` | string | Explicit robot name (e.g., `swarmjungle`). Leave empty to auto-discover `/<robot>/cmd_vel`[cite: 3, 4]. |
| `long_distance` | `1.5` | float | Distance (meters) for the **1st** and **3rd** straight legs. |
| `short_distance` | `1.0` | float | Distance (meters) for the **2nd** (shorter) straight leg. |
| `linear_speed` | `0.25` | float | Forward `linear.x` velocity command. Must be `> 0`. |
| `angular_speed` | `0.8` | float | Magnitude of `angular.z` during 90° turns. Must be `> 0`. |
| `drift_angular_trim` | `-0.05` | float | Constant `angular.z` trim during straights. **Negative pulls right** to counter left drift; positive pulls left. |
| `drift_strafe_trim` | `0.0` | float | Optional `linear.y` trim sent during straights (forwarded by the bridge[cite: 3]). |
| `distance_scale` | `1.0` | float | Multiplier on straight durations (`desired_distance / actual_distance`). |
| `turn_scale` | `1.5` | float | Multiplier on 90° turn durations. Default `1.5` compensates for `mqtt_ros2_bridge.py` dividing `angular.z` by `1.5`[cite: 3]. |
| `pause_duration` | `0.4` | float | Seconds to command `0 0 0` between each straight and turn to kill momentum. |
| `start_delay` | `2.0` | float | Seconds to wait after topic discovery before starting Leg 1. |
| `publish_rate` | `20.0` | float | *(Node param only)* Control loop rate in Hz. Keep `>= 4.0` Hz to avoid the bridge's `0.5 s` timeout[cite: 3]. |

---

## 4. Quick Track Calibration

Tune parameters in this order on the arena floor:

1. **Counteract Left Drift (`drift_angular_trim`):**
   * Still drifting left → make more negative (e.g., `-0.05` → `-0.08`).
   * Pulling right → move closer to `0.0`.
2. **Match Straight Distance (`distance_scale`):**
   * Measure actual distance traveled on Leg 1, then set `distance_scale = desired_distance / actual_distance`.
3. **Dial In 90° Turns (`turn_scale`):**
   * Under-turning (< 90°) → increase `turn_scale` (e.g., `1.5` → `1.7`).
   * Over-turning (> 90°) → decrease `turn_scale` (e.g., `1.5` → `1.3`).

---

## 5. Utility & Verification Scripts

### Dry-Run Without a Robot
Verify topic output and turn signs locally before putting the Moorebot on the floor.

```bash
# Terminal 1: Watch published Twist messages
source /opt/ros/humble/setup.bash
ros2 topic echo /testbot/cmd_vel
```

```bash
# Terminal 2: Run a quick simulated sequence
source /opt/ros/humble/setup.bash
source ~/moorebot_ws/install/setup.bash
ros2 launch moorebot_race race_fallback.launch.py \
  robot_name:=testbot \
  start_side:=left \
  long_distance:=0.5 \
  short_distance:=0.3 \
  start_delay:=1.0
```

### List All Launch Arguments
```bash
source /opt/ros/humble/setup.bash
source ~/moorebot_ws/install/setup.bash
ros2 launch moorebot_race race_fallback.launch.py --show-args
```

### Manual Keyboard Teleop (Repositioning the Robot)
Replace `swarmrobot` with the discovered robot name[cite: 4]:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/swarmrobot/cmd_vel
```
Controls: `i` forward, `,` stop, `j` turn left, `l` turn right, `Ctrl+C` exit[cite: 4].

### MQTT Telemetry Dashboard
Check live sensor rates (`battery`, `imu`, `light`, `tof`) and MQTT connection health[cite: 3, 4, 6]:

```bash
cd ~/moorebot_humble_all_in_one
bash start_dashboard.sh
```
*(Close the ROS 2 camera node before clicking **Open Camera** in the dashboard, as the camera TCP server only supports one active viewer at a time[cite: 4].)*

---

## 6. Package Structure

```text
moorebot_race/
├── launch/
│   └── race_fallback.launch.py   # Launch file exposing all configurable parameters
├── moorebot_race/
│   ├── __init__.py
│   └── race_fallback_node.py     # Auto-discovering ROS 2 Humble open-loop controller
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```