import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from random import randint

class SpeedPublisherNode(Node):
    def __init__(self):
        super().__init__("speed_publsiher")
        self.publisher  = self.create_publisher(Twist,"cmd_vel",10)
        self.create_timer(0.2,self.publish_speed)

    def publish_speed(self):
        new_msg = Twist()
        new_msg.linear.x = float(randint(0,10))
        new_msg.linear.y = float(randint(0,10))
        new_msg.angular.z = float(randint(0,10))
        self.publisher.publish(new_msg)
    

def main():
    rclpy.init()
    node = SpeedPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()