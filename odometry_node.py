import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

from kinematics.robot_kinematics import (
    DiffDriveKinematics,
    MecanumKinematics,
    ThreeWheelOmniKinematics,
    FourWheelOmniKinematics
)

from tf2_ros import TransformBroadcaster

class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__("wheel_odometry_node")
       

        self.declare_parameters(
            namespace="",
            parameters=[
                ("drive_type", "diff"),
                ("track_width", 0.5),
                ("wheelbase", 1.0),
                ("wheel_radius", 1.0)
            ]
        )

        self.drive_type = self.get_parameter("drive_type").value
        self.track_width = self.get_parameter("track_width").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.wheel_radius = self.get_parameter("wheel_radius").value

        self.set_kinematics()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.previous_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            Float64MultiArray,
            "/encoder_speeds",
            self.encoder_callback,
            10
        )

        self.publisher = self.create_publisher(
            Odometry,
            "/odom",
            10
        )
        

    def set_kinematics(self):
        L = self.track_width
        W = self.wheelbase
        R = self.wheel_radius

        if self.drive_type == "diff":
            self.kinematics = DiffDriveKinematics(L, R, W)
        elif self.drive_type == "mecanum":
            self.kinematics = MecanumKinematics(L, R, W)
        elif self.drive_type == "three_wheel_omni":
            self.kinematics = ThreeWheelOmniKinematics(L, R, W)
        elif self.drive_type == "four_wheel_omni":
            self.kinematics = FourWheelOmniKinematics(L, R, W)
        else:
            self.get_logger().error(
                f"Unsupported drive type: {self.drive_type}"
            )
            self.kinematics = None

    def encoder_callback(self, msg):
        if self.kinematics is None:
            return

        current_time = self.get_clock().now()
        dt = (
            current_time - self.previous_time
        ).nanoseconds / 1e9

        self.previous_time = current_time

        if dt <= 0:
            return

        wheel_speeds = np.array(msg.data)

        vx, vy, wz = self.kinematics.forward(wheel_speeds)

        self.x += (
            vx * np.cos(self.theta)
            - vy * np.sin(self.theta)
        ) * dt

        self.y += (
            vx * np.sin(self.theta)
            + vy * np.cos(self.theta)
        ) * dt

        self.theta += wz * dt

        self.publish_odometry(
            current_time,
            vx,
            vy,
            wz
        )

    def publish_odometry(self, time, vx, vy, wz):
        odom = Odometry()

        odom.header.stamp = time.to_msg()
        odom.header.frame_id = "odom"

        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = np.sin(self.theta / 2)
        odom.pose.pose.orientation.w = np.cos(self.theta / 2)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = wz

        self.publisher.publish(odom)
  # Broadcast TF
        self.publish_tf(current_time)

    def publish_tf(self, current_time):

        transform = TransformStamped()

        # Header
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"

        # Child frame
        transform.child_frame_id = "base_link"

        # Translation
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        # Rotation
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0

        transform.transform.rotation.z = np.sin(
            self.theta / 2.0
        )

        transform.transform.rotation.w = np.cos(
            self.theta / 2.0
        )

        # Broadcast
        self.tf_broadcaster.sendTransform(transform)



def main():
    rclpy.init()
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()