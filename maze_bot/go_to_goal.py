import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, asin
from geometry_msgs.msg import Point
import sys


class robot_go_to_goal(Node):

    def __init__(self):
        super().__init__('goal_movement_node')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.2  # seconds
        self.pose_sub = self.create_subscription(
            Odometry, '/odom', self.pose_callback, 10)
        self.timer = self.create_timer(timer_period, self.go_to_goal_function)
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
        

    def pose_callback(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation

        (roll, pitch, self.robot_pose.z) = self.euler_from_quaternion(
            quaternion.x, quaternion.y, quaternion.z, quaternion.w)


    def go_to_goal_function(self):
        self.goal_pose.x = float(sys.argv[1])
        self.goal_pose.y = float(sys.argv[2])
        self.angle_offset = float(sys.argv[3])

        x2 = self.goal_pose.x
        x1 = self.robot_pose.x

        y2 = self.goal_pose.y
        y1 = self.robot_pose.y

        self.distance_to_goal = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))
        self.angle_to_goal = atan2(y2 - y1, x2 - x1) + self.angle_offset

        angle_to_turn = self.angle_to_goal - self.robot_pose.z

        
        self.vel_msg.angular.z = angle_to_turn * 2  
        self.vel_msg.linear.x = self.distance_to_goal * 0.5

        if self.distance_to_goal < 0.05:
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
            
        
        self.vel_pub.publish(self.vel_msg)

        msg = 'DTG:{:3f} ATT:{:3f}'.format(self.distance_to_goal, angle_to_turn)
        self.get_logger().info(msg)
        


    def euler_from_quaternion(self, x, y, z, w):
        roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = asin(2.0 * (w * y - z * x))
        yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)

    gtg_node = robot_go_to_goal()

    rclpy.spin(gtg_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gtg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
