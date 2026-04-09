# PID Tuning Guide — DroneTrackAI (Team NOS)

Always tune in this exact order: **Yaw → Altitude → Forward**

---

## Monitor errors while tuning

```bash
# Terminal 1: Run simulation
bash run.sh

# Terminal 2: Watch errors live
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic echo /pid/errors
# x = yaw error | y = altitude error | z = forward error

# Terminal 3: Plot graph
ros2 run rqt_plot rqt_plot /pid/errors/x /pid/errors/y /pid/errors/z
```

---

## Step 1 — Tune Yaw (horizontal centering)

Zero other loops first:
```bash
ros2 param set /pid_controller alt.kp 0.0
ros2 param set /pid_controller fwd.kp 0.0
```

Increase yaw Kp until drone centers the target:
```bash
ros2 param set /pid_controller yaw.kp 0.5   # start
ros2 param set /pid_controller yaw.kp 0.8   # increase if too slow
ros2 param set /pid_controller yaw.kd 0.15  # add derivative to stop oscillation
ros2 param set /pid_controller yaw.ki 0.02  # add integral only if steady-state offset
```

✅ **Good yaw**: drone smoothly centers target horizontally, no oscillation.

---

## Step 2 — Tune Altitude (vertical centering)

```bash
ros2 param set /pid_controller alt.kp 0.5
ros2 param set /pid_controller alt.kd 0.10
ros2 param set /pid_controller alt.ki 0.01  # only if drone holds wrong height
```

✅ **Good altitude**: target stays at vertical center of frame.

---

## Step 3 — Tune Forward (distance keeping)

```bash
ros2 param set /pid_controller fwd.kp 1.0
ros2 param set /pid_controller fwd.kd 0.20
ros2 param set /pid_controller target_area 0.04   # desired bbox area (0=far, 0.1=close)
```

Adjust following distance:
```bash
ros2 param set /pid_controller target_area 0.06   # fly closer
ros2 param set /pid_controller target_area 0.02   # fly further
```

---

## Default Tuned Gains (Simulation)

```yaml
yaw:  kp=0.8  ki=0.02  kd=0.15   max_rate=1.2 rad/s
alt:  kp=0.6  ki=0.01  kd=0.10   max_rate=1.0 m/s
fwd:  kp=1.2  ki=0.05  kd=0.20   max_speed=3.0 m/s
target_area: 0.04
```

Save tuned gains to `config/params.yaml` for persistence.
