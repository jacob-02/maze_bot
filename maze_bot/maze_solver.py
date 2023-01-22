import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from .bot_localization import bot_localizer
from .bot_mapping import bot_mapper
from .bot_planning import bot_pathplanner
from .bot_motionplanning import bot_motionplanner


class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver', parameter_overrides=[])

        self.localizer = bot_localizer()
        self.mapper = bot_mapper()
        self.planner = bot_pathplanner()
        self.motion_planner = bot_motionplanner()

        self.subscriber = self.create_subscription(
            Image,
            '/upper_camera/image_raw',
            self.listener_callback,
            10)
        self.subscriber  # prevent unused variable warning
        self.pose_subscriber = self.create_subscription(
            Odometry, '/odom', self.motion_planner.get_pose, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.maze_solving)

        self.bridge = CvBridge()
        self.sat_view = np.zeros((100, 100))
        self.vel_msg = Twist()

    def listener_callback(self, msg):
        cv2_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.sat_view = cv2_img
        cv2.waitKey(1)

    def maze_solving(self):
        frame_disp = self.sat_view.copy()
        self.localizer.localize_bot(self.sat_view, frame_disp=frame_disp)
        self.mapper.graphify(self.localizer.maze_og)

        self.planner.find_path_nd_display(
            self.mapper.Graph.graph, self.mapper.Graph.start, self.mapper.Graph.end, self.mapper.maze, method="dijkstra")

        # cv2.waitKey(0)

        bot_loc = self.localizer.loc_car
        path = self.planner.path_to_goal
        self.motion_planner.nav_path(bot_loc, path, self.vel_msg, self.publisher_)

        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0

        self.publisher_.publish(self.vel_msg)


def main(args=None):
    rclpy.init(args=args)

    maze_solver = MazeSolver()

    rclpy.spin(maze_solver)

    maze_solver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
