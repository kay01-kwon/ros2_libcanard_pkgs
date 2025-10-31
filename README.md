# ros2_libcanard

## 1. Install


```
mkdir -p ~/esc_ws/src
```

Navigate to ~/esc_ws/src

```
cd ~/esc_ws/src
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
colcon build --packages-select ros2_libcanard --symlink-install
```

```
source ~/ros2_ws/install/setup.bash
```

Build ros2_libcanard_test

Make sure that you have already installed mavros pkg,

because it depends on mavros_msgs.

```
colcon build --packages-select ros2_libcanard_test --symlink-install
```

## CAN network setup

Make the can_setup.bash file executable.

For Jetson with can transciever

Make the bash file executable.

```
sudo chmod +x setup_can.bash
```

For slcan sincle esc
```
sudo chmod +x setup_slcan.bash
```


## Launch the launch.py

slcan_single_esc_bringup.launch.py file (Singe esc test):

```
        parameters=[
            {"interface_name": "slcan0"},
            {"num_esc": 1}
        ]
```

```
ros2 launch ros2_libcanard slcan_single_esc_bringup.launch.py
```

esc_bringup.launch.py file (Hexa copter):

```
        parameters=[
            {"interface_name": "can0"},
            {"num_esc": 6}
        ]
```


```
ros2 launch ros2_libcanard esc_bringup.launch.py
```