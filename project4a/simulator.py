import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import sin, cos, pi
import numpy as np

class Simulator(Node):
    def __init__(self):

        super().__init__('simulator')

        self.declare_parameter('wheel_distance')
        self.L = self.get_parameter('wheel_distance').value 

        self.declare_parameter('error_variance_left')
        self.error_variance_left = self.get_parameter('error_variance_left').value 

        self.declare_parameter('error_variance_right')
        self.error_variance_right = self.get_parameter('error_variance_right').value 

        self.declare_parameter('error_update_rate')
        self.error_update_rate = self.get_parameter('error_update_rate').value 

        self.init_robot_state()

        self.vl_subscriber = self.create_subscription(Float64, '/vl', self.vl_callback, 10)
        self.vr_subscriber = self.create_subscription(Float64, '/vr', self.vr_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose_update_timer = self.create_timer(0.1, self.update_pose)
        self.error_update_timer = self.create_timer(self.error_update_rate, self.update_error)


    def update_error(self):
        now = self.get_clock().now()
        dt = (now - self.last_error_update_time).nanoseconds /1e9

        self.get_logger().info(f'{dt = }')
        self.error_left = np.random.normal(self.error_left, np.sqrt(self.error_variance_left))
        self.error_right = np.random.normal(self.error_right, np.sqrt(self.error_variance_right))
        self.last_error_update_time = self.get_clock().now()

    def vl_callback(self, vl: Float64):
        self.get_logger().info(f'{vl.data = }')
        self.vl = vl.data * self.error_left
        self.get_logger().info(f'{self.vl = }')
        self.get_logger().info(f'{self.error_left = }')
        self.last_update_time = self.get_clock().now()

    def vr_callback(self, vr: Float64):
        self.get_logger().info(f'{vr.data = }')
        self.vr = vr.data * self.error_right
        self.get_logger().info(f'{self.vr = }')
        self.get_logger().info(f'{self.error_right = }')
        self.last_update_time = self.get_clock().now()

    def init_robot_state(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.vl = 0.0
        self.vr = 0.0
        self.last_update_time = self.get_clock().now()
        
        self.error_left = 1.0
        self.error_right = 1.0
        self.last_error_update_time = self.get_clock().now()

        
    def update_pose(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9

        if dt > 1.0:
            self.vl = 0.0
            self.vr = 0.0

        if self.vr == self.vl:
            v = (self.vr + self.vl)/2
            w = (self.vr - self.vl)/self.L

            self.x += v * cos(self.theta) * dt 
            self.y += v * sin(self.theta) * dt
            self.theta += w *dt
        
        else:
            icc_omega = (self.vr - self.vl)/self.L    
            icc_R = (self.L/2)*(self.vr + self.vl)/(self.vr - self.vl)
            cx = self.x - icc_R * sin(self.theta)
            cy = self.y + icc_R * cos(self.theta)

            print(f"{icc_omega = }")
            print(f"{icc_R = }")

            A = np.array([[cos(icc_omega * dt), -sin(icc_omega * dt), 0],
                        [sin(icc_omega * dt), cos(icc_omega * dt), 0],
                        [0, 0, 1]])
            
            B = np.array([[self.x - cx],
                        [self.y - cy],
                        [self.theta]])
            
            C = np.array([[cx],
                        [cy],
                        [icc_omega * dt]])
            
            updated_pose = np.dot(A, B) + C
            print(f"{updated_pose.shape = }")

            self.x = updated_pose[0][0]
            self.y = updated_pose[1][0]
            self.theta = updated_pose[2][0]

        if self.theta > pi:
            self.theta -= 2*pi
        elif self.theta < -pi:
            self.theta += 2*pi

        self.broadcast_tf() 

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        

        self.tf_broadcaster.sendTransform(t)


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def main(args = None):
    rclpy.init(args = args)
    node = Simulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
