import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import cv2
from cv_bridge import CvBridge
import numpy as np


def load_camera_info_from_yaml(yaml_url: str) -> CameraInfo:
    """Parses ROS camera calibration YAML file into sensor_msgs.msg.CameraInfo."""
    if not yaml_url:
        return None

    clean_path = yaml_url.replace('file://', '')
    if not os.path.exists(clean_path):
        return None

    try:
        with open(clean_path, 'r') as f:
            calib_data = yaml.safe_load(f)

        ci = CameraInfo()
        ci.width = int(calib_data.get('image_width', 0))
        ci.height = int(calib_data.get('image_height', 0))
        ci.distortion_model = str(calib_data.get('distortion_model', 'plumb_bob'))

        dist_coeff = calib_data.get('distortion_coefficients', {})
        if isinstance(dist_coeff, dict):
            ci.d = [float(x) for x in dist_coeff.get('data', [])]

        cam_mat = calib_data.get('camera_matrix', {})
        if isinstance(cam_mat, dict):
            ci.k = [float(x) for x in cam_mat.get('data', [])]

        rect_mat = calib_data.get('rectification_matrix', {})
        if isinstance(rect_mat, dict):
            ci.r = [float(x) for x in rect_mat.get('data', [])]

        proj_mat = calib_data.get('projection_matrix', {})
        if isinstance(proj_mat, dict):
            ci.p = [float(x) for x in proj_mat.get('data', [])]

        return ci
    except Exception as e:
        print(f"Failed to parse camera calibration YAML ({yaml_url}): {e}")
        return None


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

        self.base_camera_info = None
        if self.camera_info_url:
            self.base_camera_info = load_camera_info_from_yaml(self.camera_info_url)
            if self.base_camera_info is not None:
                self.get_logger().info(f"Loaded camera calibration from: {self.camera_info_url}")
            else:
                self.get_logger().warn(f"Failed to load camera calibration from URL: {self.camera_info_url}")

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

        if self.base_camera_info is not None:
            info_msg = CameraInfo()
            info_msg.header = raw_msg.header
            info_msg.height = h
            info_msg.width = w
            info_msg.distortion_model = self.base_camera_info.distortion_model
            info_msg.d = self.base_camera_info.d
            info_msg.k = self.base_camera_info.k
            info_msg.r = self.base_camera_info.r
            info_msg.p = self.base_camera_info.p
            self.pub_info.publish(info_msg)
        else:
            info_msg = CameraInfo()
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

