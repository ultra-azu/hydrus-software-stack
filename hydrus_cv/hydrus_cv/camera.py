import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebCam(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_frame', 10)
        self.timer = self.create_timer(0.1, self.publish_frame)
        self.cap = cv2.VideoCapture(0)  # Open the default camera (0)
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            rclpy.shutdown()

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image (BGR) to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing camera frame')
        else:
            self.get_logger().error('Failed to capture frame')


def main(args=None):
    rclpy.init(args=args)
    node = WebCam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    node.cap.release()

if __name__ == '__main__':
    main()
