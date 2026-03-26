# nortek_nucleus_ros_interface

ROS 2 interface node for the Nortek Nucleus sensor. Wraps `nortek_nucleus_driver` and publishes sensor data as standard ROS messages. Each publisher can be enabled/disabled via parameters.

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `imu/data_raw` | `sensor_msgs/Imu` | Accelerometer + gyroscope (no orientation) |
| `imu/data` | `sensor_msgs/Imu` | AHRS orientation quaternion |
| `imu/mag` | `sensor_msgs/MagneticField` | Magnetometer (Tesla) |
| `nucleus/dvl` | `geometry_msgs/TwistWithCovarianceStamped` | Bottom track velocity + uncertainty |
| `nucleus/pressure` | `sensor_msgs/FluidPressure` | Pressure (Pascal) |

## Build

Requires `nortek_nucleus_driver` to be installed (`sudo make install` from its build directory).

```bash
colcon build --packages-select nortek_nucleus_ros_interface --symlink-install
```

## Launch

```bash
ros2 launch nortek_nucleus_ros_interface nortek_nucleus_ros_interface.launch.py
```

Configuration in `config/nortek_nucleus_ros_interface_params.yaml`.
