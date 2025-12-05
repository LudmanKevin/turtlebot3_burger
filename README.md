# turtlebot3_burger

### Működés leírása
Turtlebot3 építése ros2 környezetben, valamint akadály kikerülő vezérlés létrehozása lidar segítségével.

### Működést szemléltető gráf

```mermaid
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
```

### Működést szemléltető videó

[![Robot demo video](https://img.youtube.com/vi/eSl_OqyTuD4/0.jpg)](https://www.youtube.com/watch?v=eSl_OqyTuD4)

### Felhasznált csomagok
Turtlebot3: Ez a csomag a robot "szíve". Ahhoz, hogy a robot egyátalán mozogni tudjon a bringup-nak futnia kell. Emellett a program készítése során fontos eszközök voltak még a csomagban (pl.: example, teleop).

hls_lfcd_lds_driver: Ez a lidar driver-je. Ennek a segítségével pörög a lidar, és gyűjti az adatokat, ami elengedhetetlen a tájékozódáshoz.

lidar_cluster_ros2: Kiegészítő a lidarhoz. Az egymáshoz közel lévő pontokat egy akadályként definiálja, így megkönnyíti annak kikerülését.

turtlebot3_burger mappa: Ebben a mappában két általunk készített csomag található: obstacle_detector és tb3_localization. Előbbi a haladáshoz és akadálykerüléshez kell, utóbbi pedig létrehoz egy /odometry/filtered topic-ot, ami lényegében /imu segítségével javítja /odom értékeit, és a robotunk ezt a topic-ot használja a navigációhoz.


### Csomagok klónozása

Ha feltételezzük, hogy a munkahelyünk a ros2_ws, akkor lépjünk be a /ros2_ws/src-be, és klónozzuk a csomagokat.

``` r
cd ~/ros2_ws/src
```

Turtlebot3:
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

turtlebot3_burger mappa:
``` r
git clone https://github.com/LudmanKevin/turtlebot3_burger.git
```

### Csomagok lefordítása

Ha megvagyunk a klónozásokkal, lépjünk vissza a ros2_ws-be, és folytassuk ezen parancsokkal.

``` r
cd ~/ros2_ws/ 
```
``` r
colcon build
```
``` r
source install/setup.bash
```
``` r
ros2 launch obstacle_detector start_robot_and_detector.launch.py
```