# Kulbu

A basic ROS implementation with Gazebo simulation.

* Utilises `diff_drive_controller`.
* `kulbu_hardware` implements `sysfs` for velocity control by PWM.
* No broadcast of `odom -> map` until SLAM is loaded.

## TODO

* [ ] Find a working solution for bumpers.
* [ ] Bro down?

## install

```
wstool merge https://raw.githubusercontent.com/kulbu/kulbu/indigo-devel/kulbu.rosinstall -t src

# Optional:
wstool merge https://raw.githubusercontent.com/kulbu/kulbu/indigo-devel/kulbu_sim.rosinstall -t src
wstool merge https://raw.githubusercontent.com/kulbu/kulbu/indigo-devel/kulbu_real.rosinstall -t src
```

### Untracked dependencies

```
sudo apt-get install ros-indigo-image-proc ros-indigo-tf ros-indigo-tf-conversions ros-indigo-tf-tools ros-indigo-eigen-conversions
sudo apt-get install libirrlicht-dev
wstool set ratslam_ros -t src --git https://github.com/mryellow/ratslam.git -v ratslam_ros
```

## Build

```
wstool update -t src
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
catkin build
```

## Usage

```
roslaunch kulbu_base sim.launch
roslaunch kulbu_base real.launch
rosrun turtlebot_teleop turtlebot_teleop_key /turtlebot_teleop/cmd_vel:=/kulbu/diff_drive_controller/cmd_vel
```

### Navigation

```
roslaunch kulbu_movebase move_base.launch
roslaunch kulbu_moveit moveit.launch
```

### SLAM

```
roslaunch kulbu_slam rtab.launch rviz:=true
rosrun rviz rviz -d `rospack find kulbu_slam`/config/nav.rviz
roslaunch kulbu_slam rtabmapviz.launch
roslaunch kulbu_slam orb_slam.launch
rosrun lsd_slam_viewer viewer && rosrun lsd_slam_core live_slam image:=/stereo_camera/left/image_raw camera_io:=/stereo_camera/left/camera_info
```

#### RatSLAM

```
roslaunch kulbu_base sim.launch use_ekf:=false
roslaunch kulbu_slam rat.launch use_rat_odom:=false use_sim_odom:=true
roslaunch kulbu_slam rat.launch use_rat_odom:=false topic_odom:=/kulbu/odometry/filtered
```

### Frontier exploration

```
roslaunch kulbu_navigation explore.launch
```

## Hardware

* https://github.com/kulbu/kulbu_hardware

### Wheels

#### Enable `sysfs` PWM

```
sudo modprobe pwm-meson npwm=2
sudo modprobe pwm-ctrl
// or add to /etc/modules
gpio export 87 out
gpio export 88 out
```

#### Manual PWM control

```
# PMW0 = GPIO pin #108
# PMW1 = GPIO pin #107
echo 0 > /sys/devices/platform/pwm-ctrl/enable0
echo 0 > /sys/devices/platform/pwm-ctrl/enable1
echo 512 > /sys/devices/platform/pwm-ctrl/duty0
echo 512 > /sys/devices/platform/pwm-ctrl/duty1
echo 100 > /sys/devices/platform/pwm-ctrl/freq0
echo 100 > /sys/devices/platform/pwm-ctrl/freq1
```

### RGBD Camera

* Assuming image frames are aligned with Asus Xtion Pro Live hardware registration.

#### Testing

```
roslaunch kulbu_base real.launch use_wheels:=false use_openni:=true
rosrun image_view image_view image:=/camera_depth/depth_registered/image_raw
```

## Misc

### Odometry calibration

* `diff_drive_controller` is a bit of a hack for multi-wheel, need to figure out fake wheel radius multiplier.

```
roslaunch kulbu_base sim.launch world:=empty
rosrun rviz rviz -d `rospack find kulbu_gazebo`/config/odom.rviz
rostopic pub -r 10 /kulbu/diff_drive_controller/cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 1.0]'
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
