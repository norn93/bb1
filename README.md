# bb1

roscore

roslaunch bb1_teleop joystick.launch

sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

Wait for a bit.

roslaunch bb1_bringup minimal.launch

roslaunch bb1_bringup status.launch
