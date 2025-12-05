import rclpy, os, sys, termios, math, numpy, signal
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry

def quaternion_to_yaw(q):
    # Konvertálja a kvaterniót yaw szöggé (radiánban)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)

        self.current_yaw = 0.0
        self.target_yaw = None
        self.jelenlegi_pozíció = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = None
        self.start_y = None
        self.z_sebeseg = 0.0
        self.x_sebeseg = 0.0
        self.alaphelyzetbeállítás = True

        self.stopping = False
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def normalize_angle(self, angle):
        # Normalizálja a szöget -pi és pi közé
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def signal_handler(self, sig, frame):
        self.stopping = True
        for _ in range(5):
            self.publish_stop()
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_node()
        rclpy.shutdown()

    def publish_stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def filter(self, msg):
        # Definiáljuk az akadály fogalmát és a lidar gyengeségei miatt kiszűrjük a 0-ás értékeket és a végteleneket
        # Visszatérünk a legközelebbi akadállyal és annak szögével
        local_min = float('inf')
        local_angle = 0.0
        for i, r in enumerate(msg.ranges):
            if r > msg.range_min and not math.isinf(r):
                if r < local_min:
                    local_min = r
                    local_angle = msg.angle_min + i * msg.angle_increment
        return local_min, numpy.rad2deg(local_angle)

    def scan_callback(self, msg: LaserScan):
        if self.stopping:
            self.publish_stop()
            return

        local_min, local_angle = self.filter(msg)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # Itt elemntjük a kezdő pozíciókat és a kezdő irányt
        if self.start_x == None:
            self.start_x = self.current_x

        if self.start_y == None:
            if self.current_y != 0.0:   
                self.start_y = self.current_y
            else:
                return
        
        if self.target_yaw == None:
            self.target_yaw = self.current_yaw

        # Itt iratjuk ki a terminálba a fontosabb értékeket
        self.get_logger().info("\033[33m----------Fontosabb értékek----------\033[0m")
        self.get_logger().info(f'Y tengely elmentett pontja: {self.start_y:.2f}')
        self.érték = abs(self.start_y - self.current_y)
        self.get_logger().info(f'Y tengely elmentett pontjától való távolság: {self.érték:.2f} m')
        self.delta_yaw = self.normalize_angle(self.target_yaw - self.current_yaw)
        self.get_logger().info(f'Elfordulás mértéke: {self.delta_yaw:.2f} rad')
        self.get_logger().info("\033[33m----------Akadály észlelés----------\033[0m")

        # Itt számolom ki a sebességeket ahhoz képest hogy milyen messze vagyunk az akadálytól
        self.x_sebeseg = (0.05*(local_min / 0.2))/2
        self.z_sebeseg = 0.3*(0.2/local_min)

        # Itt történik a tényleges akadály elkerülés logika
        if local_min <= 0.3 and self.alaphelyzetbeállítás: #ha van akadály

            if 0.12 <= local_min <= 0.3 and 0 < local_angle < 60:
                cmd.twist.linear.x = self.x_sebeseg
                cmd.twist.angular.z = -self.z_sebeseg

            elif 0.12 <= local_min <= 0.3 and 300 < local_angle < 360:
                cmd.twist.linear.x = self.x_sebeseg
                cmd.twist.angular.z = self.z_sebeseg

            elif 0.12 <= local_min <= 0.3 and 60 < local_angle < 180:
                cmd.twist.linear.x = 0.05
                cmd.twist.angular.z = 0.3

            elif 0.12 <= local_min <= 0.3 and 180 < local_angle < 300:
                cmd.twist.linear.x = 0.05
                cmd.twist.angular.z = -0.3

        if local_min > 0.3 or (local_min < 0.3 and 160 < local_angle < 200 ) and self.alaphelyzetbeállítás : #ha mögötte van az akadály és nincs közvetlenül előtte
        
                cmd.twist.linear.x = 0.1
                cmd.twist.angular.z = 0.0

        if 0 < abs(self.current_y - self.start_y) < 0.01 and not (0 < abs(self.delta_yaw) < 0.5) or not self.alaphelyzetbeállítás: #ha visszaért az y pozícióba és nincs alaphelyzetben

            #Odom alaphelyzetbe állítása
            self.alaphelyzetbeállítás = False
            if -0.1 < self.target_yaw - self.current_yaw < 0.1:
                self.alaphelyzetbeállítás = True
            
            elif abs(math.pi - (self.delta_yaw)) < abs(-math.pi - (self.delta_yaw)):
                cmd.twist.angular.z = 0.3
            elif abs(math.pi - (self.delta_yaw)) > abs(-math.pi - (self.delta_yaw)):
                cmd.twist.angular.z = -0.3

        self.get_logger().info(f'Akadály: {local_min:.2f} m | Szög: {local_angle:.2f}°')
        self.get_logger().info(" ")
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.signal_handler(None, None)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()