import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CamPub(Node):
    def __init__(self):
        super().__init__('cam_pub')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        # For Mac test, pass a video path via param "source"; on Linux, 0 would be webcam.
        src = self.declare_parameter('source', '').get_parameter_value().string_value
        self.cap = cv2.VideoCapture(0 if src == '' else src)
        self.timer = self.create_timer(1/15.0, self.tick)  # ~15 FPS

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('No frame')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CamPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
