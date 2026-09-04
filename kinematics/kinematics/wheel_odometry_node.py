#!/usr/bin/env python3
"""
1st things 1st, what's the purpose of this file?
Every time real wheel-speed data comes in, this node figures out how the robot actually moved, keeps a 
running best-guess of where the robot is in the world & reports that guess to the rest of the system
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from kinematics.robot_kinematics import (
    DiffDriveKinematics,
    MecanumKinematics,
    ThreeWheelOmniKinematics,
    FourWheelOmniKinematics,
)
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


# ROS 2 doesn't let us describe which way smth is facing with an angle only... every orientation field in 
# every ROS 2 msg needs a quaternion (4-num format (x, y, z, w))
# As our robot is flat on the ground & can only spin around 1 axis, x & y r always 0, and only z & w carry
# the angle info


def yaw_to_quaternion(yaw: float) -> Quaternion: # Convert 2D heading angle to 4-num quaternion
                                                 # This fn is abt orientation only (which dir the nose is pointing)
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__("wheel_odometry_node")
        self.declare_parameters( # Setup
            namespace='',
            parameters=[
                ("drive_type", "diff"),
                ("track_width", 0.5),
                ("wheelbase", 1.0),
                ("wheel_radius", 1.0),
            ]
        )
        self.drive_type, self.track_width, self.wheel_base, self.wheel_radius = (
            self.get_parameters(['drive_type', 'track_width', 'wheelbase', 'wheel_radius'])
        )

        self.set_Kinematics()

        # Starting state: "I'm at the origin, facing forward, not moving, zero time has passed yet"
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0   # Which way I'm facing
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self._last_time = self.get_clock().now() # How much time has passed between updates
        self._first_callback = True     # This flag is here to fix a bug, being which -> When we 
                                        # tested for the 1st time, it measured the time since the node started not the last update.

        self.subscription = self.create_subscription(   # Starr listening to the wheel data
            Float64MultiArray, "/encoder_speeds", self.encoder_callback, 10
        )

        self.publisher = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)


    def set_Kinematics(self):
        d_type = self.drive_type.value
        L = self.track_width.value
        W = self.wheel_base.value
        R = self.wheel_radius.value
        if d_type == "diff":
            self.kinematics = DiffDriveKinematics(L, R, W)
        elif d_type == "mecanum":
            self.kinematics = MecanumKinematics(L, R, W)
        elif d_type == "three_wheel_omni":
            self.kinematics = ThreeWheelOmniKinematics(L, R, W)
        elif d_type == "four_wheel_omni":
            self.kinematics = FourWheelOmniKinematics(L, R, W)
        else: 
            self.get_logger().error(f"Unsupported drive type: {d_type}")
            self.kinematics = None


    def encoder_callback(self, msg: Float64MultiArray): # This runs auto once per incoming /encoder_speeds msg
        if self.kinematics is None: 
            self.get_logger().error("Kinematics engine isn't initialized")
            return
        
        wheel_speeds = msg.data
        if len(wheel_speeds) != 4:
            self.get_logger().error(f"Expecting 4 wheel speeds & got {len(wheel_speeds)}.")
            return

        Vx, Vy, wz = self.kinematics.forward(wheel_speeds)  # We send the robot_kinematics.py the 4 wheel speeds &
                                                            # get back 3 nums: how fast the robot is moving, how fast 
                                                            # sideways, and how fast it's turning
        self.vx, self.vy, self.wz = float(Vx), float(Vy), float(wz)
        now = self.get_clock().now()
        if self._first_callback:
            self._first_callback = False    # Fixing the big mentioned earlier (the 1st actual msg received not since node started)
            self._last_time = now
            return

        dt = (now - self._last_time).nanoseconds / 1e9  # Vel alone wouldn't tell us how far the robot moved
                                                        # We need v*d to get dt (in sec) since the last update
        self._last_time = now
        if dt <= 0.0:   # In case for example 2 msgs arrive at the same timestamp... skip
            return

        # Vx/Vy describe motion relative to the robot's own nose, so "forward" always means wtv way the 
        # robot is facing atm (not a fixed world dir).
        # In order to update the fixed wrld pos, we must 1st rotate it by the robot's current heading
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self.publish_odometry(now, self.vx, self.vy, self.wz)  # Hand off the updates to the radio operator


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
        self.publish_tf(time)

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