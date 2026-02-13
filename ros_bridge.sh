#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/ros2_workspace/install/setup.bash

export GZ_IP=127.0.0.1
echo "Bridge starting..."

# Note: We bridge IMU, GPS, Optical Flow, and Camera data.
ros2 run ros_gz_bridge parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
    /world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU \
    /world/default/model/x500_depth_0/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat \
    /world/default/model/x500_depth_0/link/flow_link/sensor/optical_flow/optical_flow@sensor_msgs/msg/Image@gz.msgs.Image \
    /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image \
    /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
    --ros-args \
    -r /world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu:=/aquila/imu \
    -r /world/default/model/x500_depth_0/link/base_link/sensor/navsat_sensor/navsat:=/aquila/gps \
    -r /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera/image_raw \
    -r /depth_camera/points:=/depth/points