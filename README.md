This package interfaces with Logitech G29 steering wheel and is configured to work with Husky and Hunter

The folder 'steering' contains the ROS nodes and has to be copied into your ros_ws/src directory. This currently only supports ROS1.

The folder 'webview' creates the interface using ROS bridge and can be in any convinient location. Once the rosbridge_websocket is launched, open index.html in any browser.


Dependencies:

```console
sudo apt install ros-<distro>-joy ros-<distro>-joystick-drivers

sudo apt install ros-<distro>-rosbridge-suite

roslaunch rosbridge_server rosbridge_websocket_launch.xml
or
ros2 launch rosbridge_server rosbridge_websocket_launch.xml 
```