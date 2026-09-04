import rclpy
from rclpy.node import Node
import kinematics.robot_kinematics


class KinematicsNode(Node):
    def __init__(self):
        super().__init__("kinematics_node")
        self.declare_parameters(
            namespace='',
            parameters=[
                ("drive_type", "diff"),
                ("track_width", 0.5),
                ("wheelbase", 1.0),
                ("wheel_radius", 1.0),
            ]
        )

        self.drive_type, self.track_width, self.wheel_base, self.wheel_radius = (
            self.get_parameters(['drive_type','track_width','wheelbase','wheel_radius'])
        )
        self.get_logger().info(self.drive_type.value)


def main():
    rclpy.init()
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
