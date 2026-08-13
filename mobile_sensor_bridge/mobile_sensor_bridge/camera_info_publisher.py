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


def load_camera_info_from_yaml(yaml_url: str):
    """Parses ROS camera calibration YAML file into sensor_msgs.msg.CameraInfo.

    Returns:
        tuple: (CameraInfo object or None, error_message or None)
    """
    if not yaml_url:
        return None, "Empty camera_info_url provided."

    clean_path = yaml_url.replace('file://', '')
    if not os.path.exists(clean_path):
        return None, f"Calibration file not found at: {clean_path}"

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
        if not ci.d:
            ci.d = [0.0] * 5

        cam_mat = calib_data.get('camera_matrix', {})
        if isinstance(cam_mat, dict):
            data = [float(x) for x in cam_mat.get('data', [])]
            if len(data) != 9:
                return None, f"camera_matrix data length is {len(data)}, expected 9."
            ci.k = data

        rect_mat = calib_data.get('rectification_matrix', {})
        if isinstance(rect_mat, dict):
            data = [float(x) for x in rect_mat.get('data', [])]
            if len(data) == 9:
                ci.r = data
            elif len(data) == 0:
                ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        proj_mat = calib_data.get('projection_matrix', {})
        if isinstance(proj_mat, dict):
            data = [float(x) for x in proj_mat.get('data', [])]
            if len(data) != 12:
                return None, f"projection_matrix data length is {len(data)}, expected 12."
            ci.p = data

        return ci, None
    except Exception as e:
        return None, f"YAML parsing error: {e}"


def parse_jpeg_size(data: bytes):
    """Fast parsing of JPEG SOF0/SOF2 dimensions without decoding pixels."""
    if not data or len(data) < 4 or data[:2] != b'\xff\xd8':
        return None, None
    i = 2
    n = len(data)
    while i < n - 9:
        if data[i] != 0xff:
            i += 1
            continue
        marker = data[i+1]
        if marker in (0xc0, 0xc1, 0xc2, 0xc3):
            h = (data[i+5] << 8) + data[i+6]
            w = (data[i+7] << 8) + data[i+8]
            return w, h
        length = (data[i+2] << 8) + data[i+3]
        i += 2 + length
    return None, None


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
        self._cached_resolution = None
        self._cached_k = None
        self._cached_p = None
        self._warned_aspect_mismatch = False

        if self.camera_info_url:
            self.base_camera_info, err = load_camera_info_from_yaml(self.camera_info_url)
            if self.base_camera_info is not None:
                self.get_logger().info(f"Loaded camera calibration from: {self.camera_info_url}")
            else:
                self.get_logger().warn(f"Failed to load camera calibration from URL ({self.camera_info_url}): {err}")
                self.get_logger().warn("/camera_info topic will NOT be published.")
        else:
            self.get_logger().info("No camera_info_url provided. /camera_info topic will NOT be published.")

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
        need_raw = self.pub_raw.get_subscription_count() > 0
        need_info = self.base_camera_info is not None and self.pub_info.get_subscription_count() > 0

        if not need_raw and not need_info:
            return

        w, h = None, None

        if need_raw:
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
        else:
            w, h = parse_jpeg_size(msg.data)

        if need_info and w is not None and h is not None:
            info_msg = CameraInfo()
            info_msg.header = msg.header
            if not info_msg.header.frame_id:
                info_msg.header.frame_id = self.frame_id_camera
            info_msg.height = h
            info_msg.width = w
            info_msg.distortion_model = self.base_camera_info.distortion_model
            info_msg.d = self.base_camera_info.d
            info_msg.r = self.base_camera_info.r

            if (w, h) != self._cached_resolution:
                self._cached_resolution = (w, h)
                bw = float(self.base_camera_info.width)
                bh = float(self.base_camera_info.height)

                if bw > 0.0 and bh > 0.0 and (w != int(bw) or h != int(bh)):
                    sx = float(w) / bw
                    sy = float(h) / bh

                    # Scale K matrix (fx, cx, fy, cy)
                    k = list(self.base_camera_info.k)
                    self._cached_k = [
                        k[0] * sx, 0.0,        k[2] * sx,
                        0.0,       k[4] * sy, k[5] * sy,
                        0.0,       0.0,        1.0
                    ]

                    # Scale P matrix (fx, cx, Tx, fy, cy, Ty)
                    p = list(self.base_camera_info.p)
                    self._cached_p = [
                        p[0] * sx, 0.0,       p[2] * sx, p[3] * sx,
                        0.0,       p[5] * sy, p[6] * sy, p[7] * sy,
                        0.0,       0.0,       1.0,       0.0
                    ]

                    calib_aspect = bw / bh
                    stream_aspect = float(w) / float(h)
                    if abs(calib_aspect - stream_aspect) > 1e-3 and not self._warned_aspect_mismatch:
                        self.get_logger().warn(
                            f"Aspect ratio mismatch: calib {int(bw)}x{int(bh)} ({calib_aspect:.3f}) vs "
                            f"stream {w}x{h} ({stream_aspect:.3f}). Intrinsics scaling assumes pure resize and may be inaccurate if cropped."
                        )
                        self._warned_aspect_mismatch = True
                else:
                    self._cached_k = self.base_camera_info.k
                    self._cached_p = self.base_camera_info.p

            info_msg.k = self._cached_k
            info_msg.p = self._cached_p

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

