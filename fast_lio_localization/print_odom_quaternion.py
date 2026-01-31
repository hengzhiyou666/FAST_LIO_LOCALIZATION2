#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomQuaternionPrinter(Node):
    """
    订阅 /Odometry 话题，以 1Hz 频率打印四元数 x, y, z, w
    """
    
    def __init__(self):
        super().__init__("odom_quaternion_printer")
        
        self.create_subscription(
            Odometry, 
            "/Odometry", 
            self.odom_callback, 
            10
        )
        
        # 用于控制打印频率（1Hz）
        self.last_print_time = self.get_clock().now()
        self.print_interval = 1.0  # 1 秒
        
        self.get_logger().info("OdomQuaternionPrinter started, printing at 1Hz")
    
    def odom_callback(self, msg: Odometry):
        current_time = self.get_clock().now()
        time_since_last_print = (current_time - self.last_print_time).nanoseconds / 1e9
        
        # 只每 1 秒打印一次
        if time_since_last_print >= self.print_interval:
            quat = msg.pose.pose.orientation
            pos = msg.pose.pose.position
            
            print(f"Orientation quaternion: x={quat.x:.6f}, y={quat.y:.6f}, z={quat.z:.6f}, w={quat.w:.6f} | "
                  f"Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
            
            self.last_print_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = OdomQuaternionPrinter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
