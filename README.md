# bb1

sudo apt install ros-melodic-yocs-cmd-vel-mux ros-melodic-robot-state-publisher ros-melodic-teleop-twist-joy ros-melodic-joy ros-melodic-diff-drive-controller ros-melodic-joint-state-controller
```
roscore
roslaunch bb1_teleop joystick.launch
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```
Wait for a bit.

To test the basics:
```
roslaunch bb1_bringup minimal.launch
```
And then once you're sure:
```
roslaunch bb1_bringup normal.launch
```
or
```
rosrun gmapping slam_gmapping scan:=scan
```
