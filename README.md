# Fallower

## How to run
Clone the repository and pull all the submodules:
```
git clone https://github.com/parvusvox/Fallower.git
cd Fallower
git submodule update --init --recursive
```

In order to update all the submodules to origin, use this
```
git submodule foreach git pull origin master
```

## Ros Dependencies Needed
```
ros-noetic-cv-bridge
ros-noetic-vision-opencv
ros-noetic-hector-slam
```

## ROS Setup 
```
roscore # run the ros master
roslaunch kill_depth rosbot_hardware_no_depth.launch # run the modified camera & lidar module without the depth processing
rosluanch rosbot_ekf all.launch # normal motor control and whatnot

```
