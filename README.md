# ROS 2 PTU Control Unit


## PTU Driver (ROS 2 Jazzy)

A ROS 2 node for controlling a FLIR Pan-Tilt Unit (PTU) using the vendor C SDK and exposing a simple ROS interface in **degrees**, not motor steps.

The node:

- Connects to the PTU via the vendor SDK (`libcpi.a` + `estrap.c`).
- Periodically publishes the **current pan/tilt angles**.
- Accepts **absolute** and **relative** pan/tilt angle commands.
- Uses an internal step–angle conversion based on the configured step mode.

---

## Repository Layout (relevant parts)

```text
ptu_driver/
  include/
    ptu_driver/
      ptu_sdk.h          # C wrapper around vendor SDK
    sdk-2.0.4/           # FLIR PTU SDK (C)
  src/
    PTU_driver.cpp  # main PTU_driver node (shown in the snippet)
```

## Build Instructions

**Build the PTU SDK**

From repo root:

```
cd ptu_driver/include/sdk-2.0.4
make
```

This must succeed and produce **libcpi.a**.

Build the ROS 2 workspace

From the root of the workspace:

```
cd <path-to>/ros-2-ws
colcon build
source install/setup.bash
```

## Running the Node

Executable: driver

Basic run:
```
ros2 run ptu_driver driver
```

Parameters:

**time_interval (int, ms)**

Publish period for current PTU pose

Default: 150

Example:

```
ros2 run ptu_driver driver
  --ros-args -p time_interval:=50
```


## Default Step Mode

In the node:
```
step_mode = CPI_STEP_HALF;
```

So the default is **CPI_STEP_HALF** (half-step mode).
All conversions between steps and angles use this step mode (via lookup tables for step sizes and limits).


## Message and Topics
**Message Type**

ptu_messages/msg/Position

```
float64 pan   # pan angle in degrees
float64 tilt  # tilt angle in degrees
```

These are angles in degrees, not raw motor steps.
The driver converts internally between degrees and steps.

**Published Topic**

get_position_absolute (ptu_messages/msg/Position)

Current PTU pose in degrees, published every **time_interval** ms.

**Subscribed Topics**

```
set_position_absolute (ptu_messages/msg/Position)
```

Move to an absolute pose (pan/tilt in degrees).

**IMPORTANT: NaN for an axis → keep its current value** (intended behavior).

```
set_position_relative (ptu_messages/msg/Position)
```

Move by a relative offset (delta pan/tilt in degrees).

**IMPORTANT: NaN for an axis → 0 offset for that axis** (intended behavior).


## Quick Usage Examples

Assuming the node is running:

```
# See current pose
ros2 topic echo /get_position_absolute

# Absolute command: pan=30°, tilt=10°
ros2 topic pub /set_position_absolute ptu_messages/msg/Position
  "{pan: 30.0, tilt: 10.0}"

# Relative command: pan +5°, tilt -2°
ros2 topic pub /set_position_relative ptu_messages/msg/Position
  "{pan: 5.0, tilt: -2.0}"
```


## ROS 2 Services

The node exposes three control services:

1. Set Control Mode

Service: /set_control_mode

Type: std_srvs/srv/SetBool


true → velocity mode (CV)

false → position mode (CI)

Example:
```
ros2 service call /set_control_mode std_srvs/srv/SetBool "{data: true}"
```

2. Reset PTU Home

Service: /reset_home

Type: std_srvs/srv/Trigger


Example:
```
ros2 service call /reset_home std_srvs/srv/Trigger "{}"
```

3. Set Step Mode

Service: /set_step_mode

Type: ptu_messages/srv/SetStepmode


SetStepmode.srv:

```
int32 step_mode   # 0=FULL,1=HALF,2=QUARTER,3=AUTO (equivalent to EIGHT)
---
bool success
```

Example:
```
ros2 service call /set_step_mode ptu_messages/srv/SetStepmode "{step_mode: 1}"
```


4. Get Step Mode

Service: /get_control_mode

Type: std_srvs/srv/Trigger

Example:
```
ros2 service call /get_control_mode std_srvs/srv/Trigger "{}"
```




## Notes

The node uses a C wrapper (ptu_sdk.h/.c) around the vendor SDK (libcpi.a, estrap.c).

If the PTU cannot be opened (wrong port, permissions, cable), the node exits at startup.