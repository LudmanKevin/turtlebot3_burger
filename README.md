# turtlebot3_burger

### Működés leírása
Turtlebot3 építése ros2 környezetben, valamint akadály kikerülő vezérlés létrehozása lidar segítségével.

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
