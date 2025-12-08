# from asyncio import sleep
from time import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile # <--- WAJIB ADA
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        
        # Subscribe ke data Lidar dengan QoS BEST EFFORT
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) # <--- KUNCI PERBAIKANNYA
            
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Wall Follower Node Started (QoS Fixed)")

    def listener_callback(self, msg):
        # Logika menerima data scan
        ranges = msg.ranges
        size = len(ranges)
        
        # Bersihkan data 'inf' (infinity) jadi 10 meter
        cleaned_ranges = [r if r != float('inf') and not math.isnan(r) else 10.0 for r in ranges]
        
        # Bagi sektor Lidar (360 derajat)
        # Asumsi: array dimulai dari kanan memutar ke kiri
        # Slice array: Kanan[0...1/3], Depan[1/3...2/3], Kiri[2/3...3/3]
        idx_1 = int(9 * size / 24)
        idx_2 = int(13 * size / 24)
        idx_max_L0 = int(5 * size / 8)
        idx_max_L1 = int(6 * size / 8)
        idx_max_R0 = int(2 * size / 8)
        idx_max_R1 = int(3 * size / 8)
        
        # Ambil jarak terdekat di tiap sektor
        right_dist = min(cleaned_ranges[0:idx_1])
        front_dist = min(cleaned_ranges[idx_1:idx_2])
        left_dist  = min(cleaned_ranges[idx_2:])
        left_max_dist = max(cleaned_ranges[idx_max_L0:idx_max_L1])  # Untuk deteksi sudut/kelokan
        right_max_dist = max(cleaned_ranges[idx_max_R0:idx_max_R1])  # Untuk deteksi sudut/kelokan
        
        cmd = Twist()
        
        # --- DEBUGGING: Lihat data di terminal ---
        self.get_logger().info(f'F: {front_dist:.2f} | L: {left_dist:.2f} | LM: {left_max_dist:.2f} | RM: {right_max_dist:.2f} | R: {right_dist:.2f}')

        # --- LOGIKA WALL FOLLOWER (Kiri) ---
        
        # 1. Ada Tembok di Depan -> Belok Kanan Tajam
        if front_dist < 0.27: # Default 0.8
            cmd.linear.x = -1.5
            cmd.angular.z = 1.0
            self.get_logger().info("OBSTACLE!")
            
        
        # 2. Tembok Kiri Hilang (Sudut/Belokan) -> Belok Kiri
        # elif left_dist > 1.2: # Default 1.2
        #     cmd.linear.x = 0.2
        #     cmd.angular.z = 0.3 
        #     self.get_logger().info("Lost Wall - Finding Left")
        elif left_dist < 0.2: # Default 0.5
            cmd.linear.x = 0.5
            cmd.angular.z = -0.3
            self.get_logger().info("Too Close - Adjusting Right")
        
        elif right_dist < 0.2: # Default 0.5
            cmd.linear.x = 0.5
            cmd.angular.z = 0.3
            self.get_logger().info("Too Close Right - Adjusting Left")

        elif left_max_dist > 0.9: # Default 1.2
            cmd.linear.x = 0.3
            cmd.angular.z = 0.8
            self.get_logger().info("Lost Wall - Finding Left")
        
        elif right_max_dist > 0.9: # Deteksi Sudut Kanan
            cmd.linear.x = 0.3
            cmd.angular.z = -0.8
            self.get_logger().info("Corner Right - Turning Right")
            
        # 3. Terlalu Dekat Tembok Kiri -> Menjauh Sedikit
            
        # 4. Jalan Lurus (Ideal)
        else:
            cmd.linear.x = 0.6 # Default 0.4
            cmd.angular.z = 0.0 
            self.get_logger().info("Following Wall")
            
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()