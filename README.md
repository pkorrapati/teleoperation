## About
This package interfaces with Logitech G29 steering wheel + pedals and is configured to work with Husky and Hunter, with or without a HUD.

## ROS1 Package Setup
The folder 'steering' is the ROS package that has to be copied into your ros_ws/src directory. This currently only supports ROS1.

```console
cd {your_ros_ws}/src
ln -s {downloads}/teleoperation/steering
ln -s {downloads}/teleoperation/steering_data
```

## GIT Package Dependencies:
This project also needs the following git packages. Choose them based on your application:

### Husky
https://github.com/husky/husky

```console
cd {downloads}
git clone https://github.com/husky/husky.git
cd husky
git switch <rosdistro>-devel

cd {your_ros_ws}/src
ln -s {downloads}/husky/husky_desktop
ln -s {downloads}/husky/husky_msgs
ln -s {downloads}/husky/husky_viz
```

### Hunter
https://github.com/agilexrobotics/hunter_ros

```console
cd {downloads}
git clone https://github.com/agilexrobotics/hunter_ros.git

cd {your_ros_ws}/src
ln -s {downloads}/hunter_ros/hunter_msgs
```

## ROS Package Dependencies:
This project depends on teleop_twist_joy, rosbridge_suite. The packages can be either installed manually or using rosdep. 

Note: Noetic does not have packages for joystick_drivers, but older versions might need them.

```console
cd {your_ros_ws}/src
rosdep update --include-eol-distros
rosdep install steering
```
or

```console
sudo apt install ros-<distro>-joy 
sudo apt install ros-<distro>-rosbridge-suite

sudo apt install ros-<distro>-joystick-drivers
```

## Updating the launch file for Force Feedback
List connected devices using the following command:

```console
cat /proc/bus/input/devices
```
1. Identify Logitech G29 in the list of devices and note its handlers Example: event7.
2. Update the value of the param "deviceName" in the launch file to: "/dev/input/event7".

## Launching
### Husky with HUD

```console
roslaunch steering husky_steering.launch
```

### Husky without HUD

```console
roslaunch steering husky_steering_no_hud.launch
```

### Hunter with HUD

```console
roslaunch steering hunter_steering.launch
```

### Hunter without HUD

```console
roslaunch steering hunter_steering_no_hud.launch
```

## HUD
The folder 'webview' creates a HUD using ROS bridge. Once the rosbridge_websocket is running, open index.html in any browser.

Optional: If you launched without HUD, use the following to start rosbridge and then open index.html.

```console
roslaunch rosbridge_server rosbridge_websocket.launch
```
or

```console
ros2 launch rosbridge_server rosbridge_websocket_launch.xml 
```

## Environment Setup
TODO: Add udev rules file

If access is not enabled for usb, it can be enabled using:

```console
sudo chmod 777 /dev/input/js0
```

Python scripts need execute privileges, use:

```console
sudo chmod +x {downloads}/teleoperation/steering/src/*.py
```

## Operation
1. Throttle maps 1:1 with velocity.
2. Left and Right Gear Shift pedals change gears. 
3. /steer_feedback topic sets the desired location of the wheel and a spring constant that is used to calulate the force experienced as the wheel deviates from the desired location.

## Acknowledgement
The steering force feedback has device initialization code inspired by the work of https://github.com/kuriatsu

## References
1. https://www.kernel.org/doc/html/latest/input/ff.html
2. https://github.com/torvalds/linux/blob/master/include/uapi/linux/input.h
3. https://stackoverflow.com/questions/33201711/how-to-send-a-rumble-effect-to-a-device-using-python-evdev
4. https://github.com/kuriatsu/ros-g29-force-feedback