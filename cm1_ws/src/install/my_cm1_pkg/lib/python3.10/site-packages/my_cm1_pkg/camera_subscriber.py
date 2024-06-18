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

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        
        # 구독 설정
        self.subscription = self.create_subscription(
            Image,
            '/custom_camera/image_raw',
            self.image_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(
            Image,
            '/processed_camera/image_raw',
            qos_profile)

        # CvBridge 초기화
        self.bridge = CvBridge()
        self.get_logger().info('Subscription and Publisher created.')

    def image_callback(self, msg):
        self.get_logger().info('Received an image!')
        self.get_logger().info(f'Received image with width: {msg.width}, height: {msg.height}')
        
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 이미지 처리 (여기서는 원본 이미지를 다시 발행합니다)
        processed_image = cv_image  # 실제 처리 로직이 여기에 들어갑니다

        # OpenCV 이미지를 ROS 이미지 메시지로 변환
        processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')

        # 퍼블리시(publish) 발행
        self.publisher.publish(processed_msg)

        # 화면에 표시
        cv2.imshow("Camera Image", processed_image)
        cv2.waitKey(1)
        
        

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



