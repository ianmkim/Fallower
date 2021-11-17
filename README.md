# Fallower

## Install Fallower
Clone the repository and pull all the submodules:
```
git clone https://github.com/parvusvox/Fallower.git
cd Fallower
git submodule update --init --recursive
```

In order to update all the submodules to master, use this
```
git submodule foreach git pull origin master
```

## Ros Dependencies Needed
```
sudo apt-get install -y ros-noetic-cv-bridge \
ros-noetic-vision-opencv \
ros-noetic-hector-slam
```

## To Run HectorSlam on ROSBOT 2.0
```
roscd hector_mapping
vim launch/mapping_default.launch
```
and change to the following:

```
  <arg name="base_frame" default="base_link"/>
  <arg name="odom_frame" default="odom"/>
  <arg name="pub_map_odom_transform" default="true"/>
  <arg name="scan_subscriber_queue_size" default="5"/>
  <arg name="scan_topic" default="scan"/>
  <arg name="map_size" default="2048"/>
```

Then, `roscd hector_slam_launch` and change `launch/tutorial.launch` into the following:
```
<launch>

  <arg name="geotiff_map_file_path" default="$(find hector_geotiff)/maps"/>

  <param name="/use_sim_time" value="false"/>

  <node pkg="rviz" type="rviz" name="rviz"
    args="-d $(find hector_slam_launch)/rviz_cfg/mapping_demo.rviz"/>

  <include file="$(find hector_mapping)/launch/mapping_default.launch"/>

  <include file="$(find hector_geotiff_launch)/launch/geotiff_mapper.launch">
    <arg name="trajectory_source_frame_name" value="scanmatcher_frame"/>
    <arg name="map_file_path" value="$(arg geotiff_map_file_path)"/>
  </include>

</launch>
```

Then to launch hector slam simply run
```
roslaunch hector_slam_launch tutorial.launch
```

## ROS Setup 
```
roscore # run the ros master
roslaunch kill_depth rosbot_hardware_no_depth.launch # run the modified camera & lidar module without the depth processing
rosluanch rosbot_ekf all.launch # normal motor control and whatnot

```

## Run Fallower
First, make sure that you have a static IP address assigned to your laptop and NOT using DHCP. Make note of this static IP and change the ROS_IP variable in setup_laptop_for_ethernet.

First, follow the installation procedure in the FallDetection repository (github.com/parvusvox/falldetection).

Then, on the laptop (or any other computing device)
```
cd Fallower
python3 FallDetection/detection_node.py
python3 controller.py
python3 fall-detection-alert-system/action_listener.py
python3 fall-detection-alert-system/map.py
```
