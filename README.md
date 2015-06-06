# Kulbu

A basic ROS implementation with Gazebo simulation.

* Utilises `diff_drive_controller`.
* `kulbu_hardware` implements `wiringPi.softTone` for velocity control.
* No broadcast of `odom -> map` until SLAM is loaded.

## TODO

* [ ] Bro down?

## install

```
wstool merge kulbu.rosinstall -t src
wstool merge kulbu_sim.rosinstall -t src
wstool merge kulbu_real.rosinstall -t src
```

## Build

```
wstool update -t src
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro indigo -y --os=debian:jessie
catkin build
```

## Usage

```
roslaunch kulbu_base sim.launch
rosrun turtlebot_teleop turtlebot_teleop_key /turtlebot_teleop/cmd_vel:=/kulbu/diff_drive_controller/cmd_vel
```

### Odometry calibration

```
roslaunch kulbu_base sim.launch world:=empty
rosrun rviz rviz -d `rospack find kulbu_gazebo`/config/odom.rviz
rostopic pub -r 10 /kulbu/diff_drive_controller/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 1.0]'
```

### Navigation

```
roslaunch kulbu_movebase move_base.launch
roslaunch kulbu_moveit moveit.launch
```

### SLAM

```
roslaunch kulbu_slam rtab.launch
rosrun rviz rviz -d `rospack find kulbu_slam`/config/nav.rviz
roslaunch kulbu_slam rtabmapviz.launch
roslaunch kulbu_slam ratslam.launch
roslaunch kulbu_slam orb_slam.launch
rosrun lsd_slam_viewer viewer && rosrun lsd_slam_core live_slam image:=/stereo_camera/left/image_raw camera_io:=/stereo_camera/left/camera_info
```

### Frontier exploration

```
roslaunch kulbu_navigation explore.launch
```

### MoveIt! configuration

* Nothing much done here, simply testing planner dependency installs.

```
roslaunch moveit_setup_assistant setup_assistant.launch
```

### Hector

```
// roslaunch kulbu_base hector.launch
rosrun hector_exploration_controller simple_exploration_controller
rosrun kulbu_control hector_to_move_base.py
```

### Camera monitor

```
rosrun image_view image_view image:=/kulbu/camera/rgb/image_color
rosrun image_view stereo_view stereo:=stereo_camera image:=image_rect
ROS_NAMESPACE=stereo_camera rosrun rqt_reconfigure rqt_reconfigure
```

### Trouble-shooting

```
rosrun tf view_frames
```
