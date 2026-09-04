import rclpy
from rclpy.node import Node
from kinematics.robot_kinematics import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


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
        self.set_Kinematics()


        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )


        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/wheel_setpoints',
            10
        )    



    def cmd_vel_callback(self, msg: Twist):
     if self.kinematics is None:
        self.get_logger().error("Kinematics engine is not initialized!")
        return
     
     vx = msg.linear.x
     vy = msg.linear.y
     wz = msg.angular.z

        
     wheel_speeds = self.kinematics.inverse(vx, vy, wz)

        
     wheels_array = Float64MultiArray()
     wheels_array.data = wheel_speeds.tolist()
     self.get_logger().info(str(wheels_array.data))
     self.publisher.publish(wheels_array)




    def set_Kinematics(self):
     d_type = self.drive_type.value
     L = self.track_width.value
     W = self.wheel_base.value
     R = self.wheel_radius.value

     if d_type  == "diff":
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
      



def main():
    rclpy.init()
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
