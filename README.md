# turtlebot3_burger

### Működés leírása
Turtlebot3 építése ros2 környezetben, valamint akadály kikerülő vezérlés létrehozása lidar segítségével.

### Működést szemléltető gráf

graph LR

    %% LiDAR
    hlds_laser_publisher(["/hlds_laser_publisher"])
    scan["/scan"]

    %% Obstacle detector
    obstacle_detector[("/obstacle_detector")]
    cmd_vel["/cmd_vel"]

    %% Turtlebot node
    turtlebot3_node[("/turtlebot3_node")]
    imu["/imu"]
    joint_states["/joint_states"]

    %% Diff drive, odom
    diff_drive["/diff_drive_controller"]
    odom["/odom"]

    %% EKF localization
    ekf["/ekf_filter_node"]
    odom_filtered["/odometry/filtered"]

    %% TF
    robot_state_publisher["/robot_state_publisher"]
    tf["/tf"]
    transform_listener["/transform_listener_impl"]

    %% Connections
    hlds_laser_publisher --> scan --> obstacle_detector --> cmd_vel --> turtlebot3_node
    turtlebot3_node --> joint_states --> robot_state_publisher --> tf
    turtlebot3_node --> imu --> ekf
    diff_drive --> odom --> ekf --> odom_filtered --> obstacle_detector
    turtlebot3_node --> diff_drive
    ekf --> tf
    transform_listener --> tf

### Felhasznált csomagok
Turtlebot3: 

### Clone the packages
``` r
cd ~/ros2_ws/src
```

Turtlebot3 alap csomag:
``` r
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
```

Lidar driver (LDS-01):
``` r
git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
```

lidar_cluster_ros2:
``` r
git clone https://github.com/jkk-research/lidar_cluster_ros2.git
```

Obstacle_detector csomag:
``` r
git clone https://github.com/LudmanKevin/turtlebot3_burger.git
```

### Build ROS 2 packages
``` r
cd ~/ros2_ws/ 
```
``` r
colcon build --packages-select turtlebot3 hls_lfcd_lds_driver lidar_cluster_ros2 obstacle_detector
```
``` r
source install/setup.bash
```
``` r
ros2 launch obstacle_detector start_robot_and_detector.launch.py
```
