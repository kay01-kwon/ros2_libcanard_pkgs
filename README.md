# ros2_libcanard

## How to build


Navigate to ~/ros2_ws

```
cd ~/ros2_ws
```

Download the ros2_libcanard pkgs
```
git clone https://github.com/kay01-kwon/ros2_libcanard_pkgs.git
```

Firstly, build the msgs pkg.
```
colcon build --packages-select ros_libcanard_msgs
```

Source the install setup.bash.
```
source ~/ros2_ws/install/setup.bash
```

Check whether the message is generated or not.

```
ros2 interface show ros2_libcanard_msgs/msg/HexaActualRpm
```

Result
```
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
int32[6] rpm
```

```
ros2 interface show ros2_libcanard_msgs/msg/HexaCmdRaw
```

Result
```
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
int16[6] rpm
```

Build the ros2_libcanard
```
colcon build --packages-select ros2_libcanard
```

```
source ~/ros2_ws/install/setup.bash
```

## How to execute the node

Make the can_setup.bash file executable.

For Jetson
```
sudo chmod +x setup_can.bash
```


For laptop
```
sudo chmod +x setup_slcan.bash
```

Configure the esc_bringup.launch.py
Line 11 ~ 14

```
        parameters=[
            {"interface_name": "slcan0"},
            {"num_esc": 1}
        ]
```

For hexacopter and jetson...

```
        parameters=[
            {"interface_name": "can0"},
            {"num_esc": 6}
        ]
```

Launch the node.

```
ros2 launch ros2_libcanard esc_bringup.launch.py
```