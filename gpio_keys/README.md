# GPIO Keys

Orange Pi 5B two-key reader for ROS 2.

## Wiring

- key1 signal: `GPIO1_B6`, Linux sysfs GPIO `46`
- key2 signal: `GPIO1_B7`, Linux sysfs GPIO `47`
- active level: `3.3V`
- inactive level: `GND`

Do not connect either signal to 5V.

## Register And Test

```bash
source /opt/ros/humble/setup.bash
cd ~/flight_ws/car
colcon build --packages-select gpio_keys
source install/setup.bash
sudo ros2 run gpio_keys register_gpio_keys.sh
ros2 launch gpio_keys gpio_keys.launch.py
```

Topics:

- `/gpio_keys/key1`: `std_msgs/msg/Bool`, true when key1 is pressed/high
- `/gpio_keys/key2`: `std_msgs/msg/Bool`, true when key2 is pressed/high
- `/gpio_keys/event`: `std_msgs/msg/String`, for example `key1_pressed`

`/gpio_keys/key1` and `/gpio_keys/key2` are published every 200 ms by default, so
they can be checked even when the key level is already stable before the node starts.

Direct GPIO check:

```bash
cat /sys/class/gpio/gpio46/value
cat /sys/class/gpio/gpio47/value
```

## Key Controlled Track Runner

Start this launch when the two keys should control `track_runner_offboard.launch.py`:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/flight_ws/car
colcon build --packages-select gpio_keys
source install/setup.bash
sudo ros2 run gpio_keys register_gpio_keys.sh
ros2 launch gpio_keys key_track_runner_offboard.launch.py
```

Behavior:

- `key1=true, key2=false`: wait 3 seconds, then start `car_bringup track_runner_offboard.launch.py`
- `key1=false, key2=true`: wait 3 seconds, then start `car_bringup track_runner_offboard.launch.py`
- `key1=true, key2=true`: publish `stop`, then kill the started launch
- `key1=false, key2=false`: publish `stop`, then kill the started launch

Optional launch arguments can be passed to the target launch through `target_args`:

```bash
ros2 launch gpio_keys key_track_runner_offboard.launch.py \
  target_args:="start_lidar:=false start_stm:=false"
```
