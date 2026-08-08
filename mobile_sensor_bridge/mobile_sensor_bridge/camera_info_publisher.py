import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraInfoPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        self.bridge = CvBridge()
        
        self.sub_compressed = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )
        
        self.pub_raw = self.create_publisher(Image, 'image_raw', qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, 'camera_info', qos_profile_sensor_data)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        self.publish_static_transforms()
        self.get_logger().info('Camera Info and Image Decompressor Node Started')

    def publish_static_transforms(self):
        now = self.get_clock().now().to_msg()
        
        # base_link -> camera_link
        tf_cam = TransformStamped()
        tf_cam.header.stamp = now
        tf_cam.header.frame_id = 'base_link'
        tf_cam.child_frame_id = 'camera_link'
        tf_cam.transform.translation.x = 0.1
        tf_cam.transform.translation.y = 0.0
        tf_cam.transform.translation.z = 0.2
        tf_cam.transform.rotation.w = 1.0

        # base_link -> imu_link
        tf_imu = TransformStamped()
        tf_imu.header.stamp = now
        tf_imu.header.frame_id = 'base_link'
        tf_imu.child_frame_id = 'imu_link'
        tf_imu.transform.translation.x = 0.0
        tf_imu.transform.translation.y = 0.0
        tf_imu.transform.translation.z = 0.1
        tf_imu.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([tf_cam, tf_imu])

    def image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_img is None:
            return

        h, w, _ = cv_img.shape
        raw_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        raw_msg.header = msg.header
        raw_msg.header.frame_id = 'camera_link'
        self.pub_raw.publish(raw_msg)

        info_msg = CameraInfo()
        info_msg.header = raw_msg.header
        info_msg.height = h
        info_msg.width = w
        info_msg.distortion_model = 'plumb_bob'
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        fx = w * 0.8
        fy = w * 0.8
        cx = w / 2.0
        cy = h / 2.0
        
        info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.pub_info.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
