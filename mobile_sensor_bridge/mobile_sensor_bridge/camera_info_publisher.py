import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
try:
    from camera_info_manager import CameraInfoManager
except ImportError:
    CameraInfoManager = None


class CameraInfoPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        self.bridge = CvBridge()

        self.declare_parameter('frame_id_camera', 'phone_camera')
        self.declare_parameter('frame_id_imu', 'phone_imu')
        self.declare_parameter('camera_info_url', '')

        self.frame_id_camera = self.get_parameter('frame_id_camera').get_parameter_value().string_value
        self.frame_id_imu = self.get_parameter('frame_id_imu').get_parameter_value().string_value
        self.camera_info_url = self.get_parameter('camera_info_url').get_parameter_value().string_value

        self.cinfo_manager = None
        if self.camera_info_url and CameraInfoManager is not None:
            self.cinfo_manager = CameraInfoManager(self, cname='phone_camera', url=self.camera_info_url)
            self.cinfo_manager.loadCameraInfo()
            if self.cinfo_manager.isCalibrated():
                self.get_logger().info(f"Loaded camera calibration from: {self.camera_info_url}")
            else:
                self.get_logger().warn(f"Failed to calibrate camera using URL: {self.camera_info_url}")

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

        # base_link -> phone_camera
        tf_cam = TransformStamped()
        tf_cam.header.stamp = now
        tf_cam.header.frame_id = 'base_link'
        tf_cam.child_frame_id = self.frame_id_camera
        tf_cam.transform.translation.x = 0.1
        tf_cam.transform.translation.y = 0.0
        tf_cam.transform.translation.z = 0.2
        tf_cam.transform.rotation.w = 1.0

        # base_link -> phone_imu
        tf_imu = TransformStamped()
        tf_imu.header.stamp = now
        tf_imu.header.frame_id = 'base_link'
        tf_imu.child_frame_id = self.frame_id_imu
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
        if not raw_msg.header.frame_id:
            raw_msg.header.frame_id = self.frame_id_camera
        self.pub_raw.publish(raw_msg)

        if self.cinfo_manager and self.cinfo_manager.isCalibrated():
            info_msg = self.cinfo_manager.getCameraInfo()
            info_msg.header = raw_msg.header
            info_msg.height = h
            info_msg.width = w
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

