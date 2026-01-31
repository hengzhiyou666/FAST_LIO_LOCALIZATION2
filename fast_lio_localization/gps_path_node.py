#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class GpsPathNode(Node):
    """
    简单的 GPS 轨迹节点：
    - 订阅 /gps_fix (sensor_msgs/NavSatFix)
    - 以第一帧 GPS 为原点构建局部 ENU 平面
    - 在 map 坐标系下发布：
        - /gps_pose : PoseStamped
        - /gps_path : Path（累积轨迹）
    """

    def __init__(self) -> None:
        super().__init__("gps_path_node")

        # 参数：订阅的话题名，可以以后在 launch 里通过参数覆盖
        # 默认使用 RTK 话题 /gps_rtk_fix
        self.declare_parameter("gps_topic", "/gps_rtk_fix")
        gps_topic = self.get_parameter("gps_topic").get_parameter_value().string_value

        self.sub = self.create_subscription(NavSatFix, gps_topic, self.cb_navsat, 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/gps_pose", 10)
        self.pub_path = self.create_publisher(Path, "/gps_path", 10)

        self.path = Path()
        self.path.header.frame_id = "map"
        self.origin = None  # (lat0, lon0, alt0)

        # 地球半径（近似值，用于小范围局部坐标转换）
        self.R_earth = 6378137.0

        # 地图坐标系与 GPS ENU 坐标系之间的偏航角补偿（单位：弧度）
        # 正值表示逆时针旋转 yaw_offset，默认 171 度
        self.declare_parameter("yaw_offset", 2.98451302091)
        self.yaw_offset = (
            self.get_parameter("yaw_offset")
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info(f"GpsPathNode started, subscribing to {gps_topic}")

    def cb_navsat(self, msg: NavSatFix) -> None:
        # 1) status < 0 通常表示无效（NO_FIX 等），直接丢弃
        if msg.status.status < 0:
            return

        # 2) 过滤掉包含 NaN / inf 的无效浮点数，避免 RViz 报
        # "topic message contained invalid floating point values" 错误
        if not all(
            math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude)
        ):
            self.get_logger().warn(
                "Received NavSatFix with non-finite value, ignoring this sample."
            )
            return

        if self.origin is None:
            self.origin = (msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info(
                f"GPS origin set to lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}"
            )

        lat0, lon0, alt0 = self.origin

        # 经度、纬度转成局部平面坐标（近似 ENU）
        dlat = math.radians(msg.latitude - lat0)
        dlon = math.radians(msg.longitude - lon0)

        x = self.R_earth * dlon * math.cos(math.radians(lat0))
        y = self.R_earth * dlat
        z = msg.altitude - alt0

        # 应用 yaw 偏置，把 GPS 轨迹旋转到与地图坐标系对齐
        cos_yaw = math.cos(self.yaw_offset)
        sin_yaw = math.sin(self.yaw_offset)
        x_rot = cos_yaw * x - sin_yaw * y
        y_rot = sin_yaw * x + cos_yaw * y
        x, y = x_rot, y_rot

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"  # 与 localization 一致，方便在 RViz 里对比
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        self.pub_pose.publish(pose)

        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)
        self.pub_path.publish(self.path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsPathNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

