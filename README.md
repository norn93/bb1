# bb1

sudo apt install ros-melodic-yocs-cmd-vel-mux ros-melodic-robot-state-publisher ros-melodic-teleop-twist-joy ros-melodic-joy

roscore

roslaunch bb1_teleop joystick.launch

sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

Wait for a bit.

roslaunch bb1_bringup minimal.launch

roslaunch bb1_bringup status.launch
