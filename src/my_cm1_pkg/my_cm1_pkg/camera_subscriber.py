import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import cv2
from cv_bridge import CvBridge
import time

qos_profile = QoSProfile(depth=10)
qos_profile.durability = QoSDurabilityPolicy.VOLATILE
qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/custom_camera/image_raw',
            self.image_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.get_logger().info('Subscription created.')

    def image_callback(self, msg):
        self.get_logger().info('Received an image!')
        self.get_logger().info(f'Received image with width: {msg.width}, height: {msg.height}')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)
        
        # Save the image to a file
        timestamp = int(time.time() * 1000)
        filename = f'camera_image_{timestamp}.jpg'
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f'Image saved as {filename}')

        print(f"Received image with width: {msg.width}, height: {msg.height}")

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

