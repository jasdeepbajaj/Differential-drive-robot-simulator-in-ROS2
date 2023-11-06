import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from disc_robot import *

robot = load_disc_robot('normal.robot')
L = robot['wheels']['distance'] #wheel_distance

class VelocityTranslator(Node):

    def __init__(self):
        super().__init__('velocity_translator_node')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.left_publisher = self.create_publisher(Float64, '/vl', 10)
        self.right_publisher = self.create_publisher(Float64, '/vr', 10)

    def cmd_vel_callback(self, msg: Twist):
        
        vel = msg.linear.x
        omega = msg.angular.z

        vel_l = (vel + omega * L / 2)
        vel_r = (vel - omega * L / 2)


        print(f"{vel_l = }")
        print(f"{vel_r = }")

        vel_l = Float64(data = vel_l)
        vel_r = Float64(data = vel_r)

        self.left_publisher.publish(vel_l)
        self.right_publisher.publish(vel_r)


def main(args = None):
    rclpy.init(args = args)
    node = VelocityTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
