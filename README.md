# ros2_libcanard

ROS2 interface for libcanard-based UAVCAN ESC control with safety features including voltage monitoring and emergency landing.

## Features

- **Multi-ESC Support**: Control 1, 4, or 6 ESCs simultaneously
- **ESC Stabilization**: 200ms initialization delay for reliable startup
- **Voltage Monitoring**: Automatic battery voltage tracking
- **Emergency Landing**: Auto-descent when battery voltage drops critically
- **Broadcast Monitoring**: Tracks and reports CAN communication failures
- **Optimized CAN Bus**: 4096-byte memory pool for reliable multi-ESC operation

## 1. Install

Create workspace:
```bash
mkdir -p ~/esc_ws/src
cd ~/esc_ws/src
```

Download the ros2_libcanard packages:
```bash
git clone https://github.com/kay01-kwon/ros2_libcanard_pkgs.git
```

Build the messages package first:
```bash
cd ~/esc_ws
colcon build --packages-select ros2_libcanard_msgs
source install/setup.bash
```

Verify message generation:
```bash
ros2 interface show ros2_libcanard_msgs/msg/HexaActualRpm
# Expected output:
# std_msgs/Header header
# int32[6] rpm
# int32[6] acceleration

ros2 interface show ros2_libcanard_msgs/msg/HexaCmdRaw
# Expected output:
# std_msgs/Header header
# int16[6] cmd_raw
```

Build the ros2_libcanard package:
```bash
colcon build --packages-select ros2_libcanard --symlink-install
source install/setup.bash
```

Build ros2_libcanard_test (requires mavros_msgs):
```bash
# Install mavros if not already installed:
# sudo apt install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-msgs

colcon build --packages-select ros2_libcanard_test --symlink-install
source install/setup.bash
```

## 2. CAN Network Setup

### For Jetson with CAN transceiver:
```bash
sudo chmod +x setup_can.bash
./setup_can.bash
```

### For SLCAN single ESC:
```bash
sudo chmod +x setup_slcan.bash
./setup_slcan.bash
```

## 3. Configuration Parameters

The following parameters can be configured in launch files or via command line:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `interface_name` | string | `"can0"` | CAN interface name |
| `num_esc` | int | `6` | Number of ESCs (1, 4, or 6) |
| `low_voltage_threshold` | double | `21.0` | Warning voltage threshold (V) |
| `critical_voltage_threshold` | double | `20.0` | Emergency landing threshold (V) |

### Battery Voltage Thresholds by Type:
- **6S LiPo (22.2V nominal)**: `low=21.0V`, `critical=20.0V`
- **4S LiPo (14.8V nominal)**: `low=14.0V`, `critical=13.2V`
- **3S LiPo (11.1V nominal)**: `low=10.5V`, `critical=9.9V`

## 4. Launch

### Single ESC Test (SLCAN):
```bash
ros2 launch ros2_libcanard slcan_single_esc_bringup.launch.py
```

### Hexacopter (6 ESCs):
```bash
ros2 launch ros2_libcanard esc_bringup.launch.py
```

### With Custom Parameters:
```bash
ros2 launch ros2_libcanard esc_bringup.launch.py \
  interface_name:=can0 \
  num_esc:=6 \
  low_voltage_threshold:=21.0 \
  critical_voltage_threshold:=20.0
```

**Expected startup messages:**
```
Waiting for ESCs to stabilize (200ms delay)...
ESCs ready - starting command timer
```

## 5. Safety Features

### Voltage Monitoring

The system continuously monitors battery voltage and provides three levels of protection:

1. **NORMAL** (V ≥ low_voltage_threshold)
   - Normal operation

2. **LOW_VOLTAGE_WARNING** (critical < V < low)
   - Warning logs published
   - Manual landing recommended

3. **CRITICAL_EMERGENCY_LANDING** (V < critical_voltage_threshold)
   - **Automatic ESC command reduction at 2% per cycle**
   - Minimum control authority maintained (100 raw cmd)
   - Continues until voltage recovers or landing completes

**Monitor voltage:**
```bash
# Real-time voltage monitoring
ros2 topic echo /voltage

# Check warnings/errors
ros2 topic echo /rosout | grep -i voltage
```

### Broadcast Failure Monitoring

Tracks CAN broadcast failures and reports consecutive failures:
```
[WARN] Failed to broadcast ESC command (consecutive failures: 5)
[INFO] Broadcast recovered after 5 failures
```

## 6. Topics

### Published Topics:
- `/uav/actual_rpm` - ESC actual RPM feedback
- `/uav/broadcast` - Command broadcast echo
- `/voltage` - Battery voltage (std_msgs/Float64)

### Subscribed Topics:
- `/uav/cmd_raw` - ESC raw command input
  - `SingleCmdRaw` for 1 ESC
  - `QuadCmdRaw` for 4 ESCs
  - `HexaCmdRaw` for 6 ESCs

## 7. Testing

### Single ESC Test:

Terminal 1 - PX4:
```bash
ros2 launch px4_launch px4.launch
```

Terminal 2 - PX4 Client:
```bash
ros2 run px4_launch px4_client_node
```

Terminal 3 - ESC Test:
```bash
ros2 launch ros2_libcanard_test single_esc_test.launch.py
```

### Hexacopter Test with RC:

Terminal 1 - PX4:
```bash
ros2 launch px4_launch px4.launch
```

Terminal 2 - PX4 Client:
```bash
ros2 run px4_launch px4_client_node
```

Terminal 3 - ESC Test:
```bash
ros2 launch ros2_libcanard_test esc_test.launch.py
```

### Manual Command Test (Props Removed for Safety):
```bash
# Low RPM test
ros2 topic pub /uav/cmd_raw ros2_libcanard_msgs/msg/HexaCmdRaw \
  "{cmd_raw: [100, 100, 100, 100, 100, 100]}"

# Individual ESC test
ros2 topic pub /uav/cmd_raw ros2_libcanard_msgs/msg/HexaCmdRaw \
  "{cmd_raw: [500, 0, 0, 0, 0, 0]}"
```

## 8. Troubleshooting

### ESCs not responding:
- Check CAN bus connection: `candump can0`
- Verify interface is up: `ip link show can0`
- Check startup messages for "ESCs ready"

### Broadcast failures:
- Check power supply capacity (6 ESCs draw significant current)
- Monitor `/rosout` for failure messages
- Verify CAN termination resistors

### Emergency landing activates unexpectedly:
- Check battery voltage: `ros2 topic echo /voltage`
- Adjust voltage thresholds for your battery type
- Verify power supply stability under load

## 9. Power Supply Considerations

**Critical for multi-ESC operation:**
- Calculate total current: `6 ESCs × max current per ESC + Jetson current`
- Add 20-30% safety margin
- Consider separate power supplies for ESCs and flight controller
- Monitor voltage drop under load

**Symptoms of insufficient power:**
- System shutdowns at high RPM
- Voltage drops below threshold
- Intermittent CAN communication failures

## 10. Architecture

```
┌─────────────────┐
│  ROS2 Node      │
│  (10ms timer)   │
└────────┬────────┘
         │ RawCmd
         ▼
┌─────────────────┐     ┌──────────────┐
│ UAVCAN/libcanard│────▶│  CAN Bus     │
│ (100μs process) │     │  (can0)      │
└─────────────────┘     └──────┬───────┘
         ▲                      │
         │ ESC Status           │
         │ Voltage              │
         └──────────────────────┘
              │
    ┌─────────┴──────────┐
    │                    │
┌───▼────┐         ┌────▼───┐
│ ESC 1  │   ...   │ ESC 6  │
└────────┘         └────────┘
```

## 11. License

[Add your license here]

## 12. Contributing

[Add contribution guidelines here]