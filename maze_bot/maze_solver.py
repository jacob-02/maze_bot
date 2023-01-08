import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from .bot_localization import bot_localizer
from .bot_mapping import bot_mapper 
import numpy as np


class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver', parameter_overrides=[])
        self.subscriber = self.create_subscription(
            Image,
            '/upper_camera/image_raw',
            self.listener_callback,
            10)
        self.subscriber  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.maze_solving)
        self.localizer = bot_localizer()
        self.mapper = bot_mapper()
        self.bridge = CvBridge()
        self.sat_view = np.zeros((100,100))

    def listener_callback(self, msg):
        cv2_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.sat_view = cv2_img
        self.get_logger().info('I heard: "%s"' % msg.header.frame_id)
        cv2.waitKey(1)

    def maze_solving(self):
        frame_disp = self.sat_view.copy()
        self.localizer.localize_bot(self.sat_view, frame_disp=frame_disp)
        self.mapper.graphify(self.localizer.maze_og)

def main(args=None):
    rclpy.init(args=args)

    maze_solver = MazeSolver()

    rclpy.spin(maze_solver)

    maze_solver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
