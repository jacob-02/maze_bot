import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os

class Video_get(Node):
    def __init__(self):
        super().__init__('video_get', parameter_overrides=[])
        self.subscriber = self.create_subscription(
            Image,
            '/upper_camera/image_raw',
            self.listener_callback,
            10)
        self.subscriber  # prevent unused variable warning
        vid_path = os.path.join(os.getcwd(), 'video.avi')
        self.out = cv2.VideoWriter(vid_path, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (1280, 720))
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv2_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('video', cv2_img)
        self.get_logger().info('I heard: "%s"' % msg.header.frame_id)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    video_get = Video_get()

    rclpy.spin(video_get)

    video_get.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()