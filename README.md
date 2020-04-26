# BB1

## Dependencies

`sudo apt install ros-melodic-yocs-cmd-vel-mux ros-melodic-robot-state-publisher ros-melodic-teleop-twist-joy ros-melodic-joy ros-melodic-diff-drive-controller ros-melodic-joint-state-controller`

## Starting

`sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger`

Wait for a bit. Then:

`roslaunch bb1_bringup normal.launch`

## SLAM/AMCL

`rosrun gmapping slam_gmapping scan:=scan`

`roslaunch bb1_bringup amcl.launch`