#!/usr/bin/env python3

import copy
import threading
import time
import numpy as np
# Fix for numpy compatibility with transforms3d
if not hasattr(np, 'float'):
    np.float = np.float64
    np.int = np.int_
    np.complex = np.complex_
    np.bool = np.bool_
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import rclpy.timer
import tf_transformations
import tf2_ros
from geometry_msgs.msg import Transform
from std_msgs.msg import Header


class TransformFusion(Node):
    def __init__(self):
        super().__init__("transform_fusion")

        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pub_localization = self.create_publisher(Odometry, "/localization", 1)

        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 1)
        self.create_subscription(Odometry, "/map_to_odom", self.cb_save_map_to_odom, 1)

        # 坐标系旋转补偿参数（单位：弧度）
        # 顺时针旋转 90 度 = -π/2，用于对齐 body 坐标系的前进方向
        self.declare_parameter("body_frame_yaw_offset", -1.5707963267948966)  # -90 度
        self.body_frame_yaw_offset = (
            self.get_parameter("body_frame_yaw_offset")
            .get_parameter_value()
            .double_value
        )
        
        # 预计算旋转矩阵（绕 Z 轴旋转）
        self.R_body_correction = np.eye(4)
        cos_yaw = np.cos(self.body_frame_yaw_offset)
        sin_yaw = np.sin(self.body_frame_yaw_offset)
        self.R_body_correction[:3, :3] = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw,  cos_yaw, 0],
            [0,        0,       1]
        ])

        self.freq_pub_localization = 50
        self.timer = self.create_timer(1/self.freq_pub_localization, self.transform_fusion)
        # threading.Thread(target=self.transform_fusion, daemon=True).start()

    def pose_to_mat(self, pose_msg):
        trans = np.eye(4)
        trans[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        quat = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans

    def transform_fusion(self):
        if self.cur_odom_to_baselink is None:
            return

        if self.cur_map_to_odom is not None:
            T_map_to_odom = self.pose_to_mat(self.cur_map_to_odom.pose.pose)
        else:
            T_map_to_odom = np.eye(4)

        transform_msg = Transform()
        transform_msg.translation.x = T_map_to_odom[0, 3]
        transform_msg.translation.y = T_map_to_odom[1, 3]
        transform_msg.translation.z = T_map_to_odom[2, 3]
        
        quat = tf_transformations.quaternion_from_matrix(T_map_to_odom)

        transform_msg.rotation.x = quat[0]
        transform_msg.rotation.y = quat[1]
        transform_msg.rotation.z = quat[2]
        transform_msg.rotation.w = quat[3]
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.cur_odom_to_baselink.header.frame_id
        
        # print(self.cur_odom_to_baselink.header)
        transform_stamped_msg = tf2_ros.TransformStamped(
                header = self.cur_odom_to_baselink.header,
                child_frame_id = "camera_init",
                transform = transform_msg
            )
        transform_stamped_msg.header.frame_id = "map"
        self.tf_broadcaster.sendTransform(transform_stamped_msg)

        cur_odom = copy.copy(self.cur_odom_to_baselink)
        if cur_odom is not None:
            T_odom_to_base_link = self.pose_to_mat(cur_odom.pose.pose)
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
            
            # 应用 body 坐标系旋转补偿，使前进方向对齐
            # 旋转只影响姿态，不影响位置
            T_map_to_base_link_corrected = np.matmul(T_map_to_base_link, self.R_body_correction)

            xyz = tf_transformations.translation_from_matrix(T_map_to_base_link_corrected)
            quat = tf_transformations.quaternion_from_matrix(T_map_to_base_link_corrected)

            localization = Odometry()
            localization.pose.pose = Pose(
                position = Point(x = xyz[0], y = xyz[1], z = xyz[2]), 
                orientation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])
            )
            localization.twist = cur_odom.twist

            localization.header.stamp = self.get_clock().now().to_msg()
            localization.header.frame_id = "map"
            localization.child_frame_id = "body"
            self.pub_localization.publish(localization)


    def cb_save_cur_odom(self, msg):
        self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg):
        self.cur_map_to_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
