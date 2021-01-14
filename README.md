# simply_diff_drive_simulator

Simulation of a simple differential wheeled robot using ignition gazebo

[![](https://img.youtube.com/vi/n9oqzNH6MQM/0.jpg)](https://www.youtube.com/watch?v=n9oqzNH6MQM)

In this video, we use amcl, neonavigation, and octomap_mapping.

## Usage

### Simulation + Navigation + Mapping

In the video above, we do the following.

terminal 1: ignition gazebo(background) + neonavigation
```Shell
roslaunch simply_diff_drive_simulator run_simulation.launch with_navigation:=true
```

terminal 2: octomap_mapping
```Shell
roslaunch simply_diff_drive_simulator octomapping.launch disable_stf:=true
```

Mark checkbox, if you want to visualize `octomap`.
![octomap image](/image/octomap_select.png)

### Simulation only

```Shell
roslaunch simply_diff_drive_simulator run_simulation.launch no_ign_gui:=false
```

After clicking ignitoin gazebo gui, you can do the following
+ W: forward
+ A: turn left
+ S: turn right
+ D: backward
+ R: stop

## Install

The following environment is recommended.

+ Ubnuntu 18.04
+ ROS melodic
+ Ignition gazebo citadel

```Shell
export IGNITION_VERSION=citadel
mkdir -p sim_ws/src
cd sim_ws/src
git clone https://github.com/ignitionrobotics/ros_ign.git -b melodic
git clone https://github.com/at-wat/neonavigation.git
git clone https://github.com/at-wat/neonavigation_msgs.git
git clone https://github.com/at-wat/neonavigation_rviz_plugins.git
git clone https://github.com/NaokiTakahashi12/simply_diff_drive_simulator.git
cd ../
rosdep install --from-paths src -i -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
```
