## About
This package interfaces with Logitech G29 steering wheel + pedals and is configured to work with Husky and Hunter, with or without a HUD.

## ROS1 Package Setup
The folder 'steering' is the ROS package that has to be copied into your ros_ws/src directory. This currently only supports ROS1.

```console
cd {your_ros_ws}/src
ln -s {downloads}/teleoperation/steering
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
sudo apt install ros-<distro>-teleop-twist-joy 
sudo apt install ros-<distro>-rosbridge-suite

sudo apt install ros-<distro>-joystick-drivers
```

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
Throttle maps 1:1 with velocity. <br />
Left and Right Gear Shift pedals change gears. 