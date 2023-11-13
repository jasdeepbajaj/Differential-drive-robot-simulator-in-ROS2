import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from math import sin, cos, pi, atan2
import numpy as np
from geometry_msgs.msg import TransformStamped
from project4a.disc_robot import load_disc_robot
import os



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


class VelocityTranslator(Node):
    """ro
    Node for translating linear and angular velocities to left and right wheel velocities.
    """
    def __init__(self):
        """
        Initializes the VelocityTranslator node.
        """
        super().__init__('combined')

        self.declare_parameter('robot_name', 'normal.robot')
        file_name = self.get_parameter('robot_name').value
        file_path = os.path.join("/home/jasdeep/ros2_ws/src/project4a/", file_name)       
        robot = load_disc_robot(file_path)        
        
        self.L = robot['wheels']['distance'] #wheel_distance     
        
        self.error_variance_left = robot['wheels']['error_variance_left']
        self.error_variance_right = robot['wheels']['error_variance_right']
        self.error_update_rate = robot['wheels']['error_update_rate']

        self.init_robot_state()
        
        #Subscribed to cmd_vel
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        #update pose
        self.timer = self.create_timer(0.1, self.update_pose)

        #TFBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        


    def cmd_vel_callback(self, msg: Twist):

        vel = msg.linear.x
        omega = msg.angular.z

        vel_l, vel_r = self.get_wheel_velocities(vel, omega)

        self.vl = vel_l
        self.vr = vel_r

        self.last_update_time = self.get_clock().now()


    def update_pose(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9

        if dt > 1.0:
            self.vl = 0.0
            self.vr = 0.0

        
        
        
        # self.x += v * cos(self.theta) * dt
        # self.y += v * sin(self.theta) * dt
        # self.theta += w * dt

        
        
        if self.vr == self.vl:
            v = (self.vr + self.vl)/2
            w = (self.vr - self.vl)/self.L

            #icc_R = float('inf')
            #cx = float('inf')
            #cy = float('inf')
            self.x += v * cos(self.theta) * dt 
            self.y += v * sin(self.theta) * dt
            self.theta += w *dt
            #self.theta = atan2(self.y, self.x)
        
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
            
            print(f"{A.shape = }")
    
            B = np.array([[self.x - cx],
                        [self.y - cy],
                        [self.theta]])
            
            print(f"{B.shape = }")
            
            C = np.array([[cx],
                        [cy],
                        [icc_omega * dt]])
            
            print(f"{C.shape = }")
            
            updated_pose = np.dot(A, B) + C
            print(f"{updated_pose.shape = }")

            self.x = updated_pose[0][0]
            self.y = updated_pose[1][0]
            self.theta = updated_pose[2][0]

        print(f"{self.x = }")
        print(f"{self.y = }")
        print(f"{self.theta = }")

        if self.theta > pi:
            self.theta -= 2*pi
        elif self.theta < -pi:
            self.theta += 2*pi

        print(f"After wrapping theta, {self.theta = }")

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

    def init_robot_state(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.vl = 0.0
        self.vr = 0.0
        
        self.error_left = 1.0
        self.error_right = 1.0
        self.last_update_time = self.get_clock().now()

    

        


    def get_wheel_velocities(self, vel, omega):
        vel_l = (vel - omega * self.L / 2)
        vel_r = (vel + omega * self.L / 2)

        print(f"{vel_l = }")
        print(f"{vel_r = }")

        return vel_l, vel_r


def main(args = None):
    rclpy.init(args = args)
    node = VelocityTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":

    main()
