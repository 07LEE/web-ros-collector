"""ROS2 Main Node for Mobile Sensor Bridge, integrating CameraBridge and ImuBridge."""

import http.server
import json
import os
import queue
import socket
import socketserver
import ssl
import subprocess
import threading
import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory
from mobile_sensor_bridge.camera_bridge import CameraBridge
from mobile_sensor_bridge.imu_bridge import ImuBridge
from mobile_sensor_bridge.battery_bridge import BatteryBridge
from mobile_sensor_bridge.gps_bridge import GpsBridge


class MobileSensorHTTPRequestHandler(http.server.BaseHTTPRequestHandler):
    """HTTPS Request Handler routing web requests to CameraBridge and ImuBridge handlers."""

    node = None
    html_filepath = ""

    def log_message(self, format, *args):
        """Suppresses default HTTP request logging to prevent terminal output spam."""
        pass

    def do_GET(self):
        """Handles HTTP GET requests to serve static web assets or perform time sync."""
        if self.path == '/sync_time':
            try:
                current_time_ns = self.node.get_clock().now().nanoseconds if self.node else 0
                response_data = json.dumps({'server_time_ns': current_time_ns}).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(response_data)
                return
            except Exception:
                self.send_response(500)
                self.end_headers()
                return

        if self.path == '/record/stop':
            try:
                if self.node is not None:
                    self.node.stop_bag_recording()
                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b"OK")
                return
            except Exception:
                self.send_response(500)
                self.end_headers()
                return

        filename = 'index.html' if self.path in ('/', '/index.html') else self.path.lstrip('/')
        web_dir = os.path.dirname(self.html_filepath)
        target_path = os.path.abspath(os.path.join(web_dir, filename))

        if os.path.commonpath([target_path, web_dir]) != web_dir:
            self.send_response(403)
            self.end_headers()
            return

        if os.path.exists(target_path) and os.path.isfile(target_path):
            try:
                with open(target_path, 'rb') as f:
                    content = f.read()

                content_type = 'text/html; charset=utf-8'
                if filename.endswith('.css'):
                    content_type = 'text/css; charset=utf-8'
                elif filename.endswith('.js'):
                    content_type = 'application/javascript; charset=utf-8'

                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.end_headers()
                self.wfile.write(content)
            except Exception as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(str(e).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def parse_stamp(self, client_ts_ms_str: str):
        """Parses client timestamp in milliseconds or falls back to node clock."""
        if self.node is not None:
            return self.node.parse_stamp(client_ts_ms_str)
        return None

    def do_POST(self):
        """Handles HTTP POST requests and routes payloads to CameraBridge or ImuBridge."""
        if self.path == '/device_info':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))

                if self.node is not None:
                    self.node.enqueue_device_info(data)

                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b"OK")
            except Exception:
                self.send_response(500)
                self.end_headers()

        elif self.path == '/imu':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))

                if self.node is not None:
                    client_ts_str = self.headers.get('X-Client-Timestamp-Ms') or str(data.get('clientTimestampMs', ''))
                    stamp = self.parse_stamp(client_ts_str)
                    self.node.enqueue_imu(data, stamp)

                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b"OK")

            except Exception:
                self.send_response(500)
                self.end_headers()

        elif self.path == '/battery':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))

                if self.node is not None:
                    client_ts_str = self.headers.get('X-Client-Timestamp-Ms') or str(data.get('clientTimestampMs', ''))
                    stamp = self.parse_stamp(client_ts_str)
                    self.node.enqueue_battery(data, stamp)

                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b"OK")

            except Exception:
                self.send_response(500)
                self.end_headers()

        elif self.path == '/gps':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))

                if self.node is not None:
                    client_ts_str = self.headers.get('X-Client-Timestamp-Ms') or str(data.get('clientTimestampMs', ''))
                    stamp = self.parse_stamp(client_ts_str)
                    self.node.enqueue_gps(data, stamp)

                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b"OK")

            except Exception:
                self.send_response(500)
                self.end_headers()

        elif self.path == '/upload':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                content_type = self.headers.get('Content-Type', '')

                if self.node is not None:
                    client_ts_str = self.headers.get('X-Client-Timestamp-Ms')
                    stamp = self.parse_stamp(client_ts_str)
                    self.node.enqueue_upload(post_data, content_type, stamp)

                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Connection', 'keep-alive')
                self.end_headers()
                self.wfile.write(b"OK")

            except Exception:
                self.send_response(500)
                self.end_headers()

        elif self.path == '/record/start':
            try:
                if self.node is not None:
                    self.node.start_bag_recording()
                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b"OK")
            except Exception:
                self.send_response(500)
                self.end_headers()

        elif self.path == '/record/stop':
            try:
                if self.node is not None:
                    self.node.stop_bag_recording()
                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b"OK")
            except Exception:
                self.send_response(500)
                self.end_headers()


class SecureHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    """Multi-threaded HTTPS server wrapping client sockets individually per connection."""
    allow_reuse_address = True

    def __init__(self, server_address, request_handler_class, ssl_context):
        super().__init__(server_address, request_handler_class)
        self.ssl_context = ssl_context

    def get_request(self):
        newsock, fromaddr = self.socket.accept()
        conn = self.ssl_context.wrap_socket(newsock, server_side=True)
        return conn, fromaddr


class MobileSensorBridgeNode(Node):
    """ROS2 Node managing HTTPS server and orchestrating CameraBridge and ImuBridge."""

    def __init__(self):
        super().__init__('mobile_sensor_bridge_node')

        # Declare ROS 2 Parameters
        self.declare_parameter('port', 8443)
        self.declare_parameter('image_topic', '/image_raw/compressed')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('battery_topic', '/robot/battery')
        self.declare_parameter('gps_topic', '/robot/gps')
        self.declare_parameter('frame_id_camera', 'phone_camera')
        self.declare_parameter('frame_id_imu', 'phone_imu')
        self.declare_parameter('frame_id_gps', 'gps_link')
        self.declare_parameter('bag_output_dir', '')
        self.declare_parameter('heartbeat_timeout', 10.0)

        self.server_port = self.get_parameter('port').get_parameter_value().integer_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        battery_topic = self.get_parameter('battery_topic').get_parameter_value().string_value
        gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        frame_id_camera = self.get_parameter('frame_id_camera').get_parameter_value().string_value
        frame_id_imu = self.get_parameter('frame_id_imu').get_parameter_value().string_value
        frame_id_gps = self.get_parameter('frame_id_gps').get_parameter_value().string_value
        custom_bag_dir = self.get_parameter('bag_output_dir').get_parameter_value().string_value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').get_parameter_value().double_value

        # Instantiate Camera, IMU, Battery, and GPS Bridges
        self.camera_bridge = CameraBridge(self, topic_name=image_topic, frame_id=frame_id_camera)
        self.imu_bridge = ImuBridge(self, topic_name=imu_topic, frame_id=frame_id_imu)
        self.battery_bridge = BatteryBridge(self, topic_name=battery_topic, frame_id=frame_id_imu)
        self.gps_bridge = GpsBridge(self, topic_name=gps_topic, frame_id=frame_id_gps)

        # Rosbag2 Automatic Recording Setup
        self.record_process = None
        self.last_data_received_time = 0.0

        if custom_bag_dir:
            self.bag_dir = os.path.abspath(custom_bag_dir)
        else:
            workspace_root = os.getcwd()
            try:
                share_dir = get_package_share_directory('mobile_sensor_bridge')
                possible_root = os.path.abspath(os.path.join(share_dir, '..', '..', '..', '..'))
                if os.path.exists(os.path.join(possible_root, 'src')) or os.path.exists(os.path.join(possible_root, 'install')):
                    workspace_root = possible_root
            except Exception:
                pass
            self.bag_dir = os.path.join(workspace_root, 'data')
        os.makedirs(self.bag_dir, exist_ok=True)

        # Separate Thread-safe Bounded Publishing Queues for Camera & Sensors
        self.camera_publish_queue = queue.Queue(maxsize=10)
        self.sensor_publish_queue = queue.Queue(maxsize=500)
        self.create_timer(0.001, self._process_publish_queues)
        self.create_timer(1.0, self._check_heartbeat_timeout)

        # Device Info Publisher
        self.device_info_publisher = self.create_publisher(
            String,
            'mobile_sensor_bridge/device_info',
            10
        )

        self.ip_addresses = self.get_all_local_ips()

        # SSL Certificates Setup
        user_ros_dir = os.path.expanduser('~/.ros')
        self.cert_dir = os.path.join(user_ros_dir, 'mobile_sensor_bridge_certs')
        os.makedirs(self.cert_dir, exist_ok=True)
        self.cert_file = os.path.join(self.cert_dir, 'cert.pem')
        self.key_file = os.path.join(self.cert_dir, 'key.pem')

        self.generate_certificates()
        if os.path.exists(self.key_file):
            try:
                os.chmod(self.key_file, 0o600)
            except Exception:
                pass

        # Web Assets Location Resolution
        try:
            share_dir = get_package_share_directory('mobile_sensor_bridge')
            self.html_filepath = os.path.join(share_dir, 'web', 'index.html')
        except Exception:
            self.html_filepath = os.path.join(
                os.getcwd(),
                'mobile_sensor_bridge',
                'web',
                'index.html'
            )

        # Configure HTTP Request Handler
        MobileSensorHTTPRequestHandler.node = self
        MobileSensorHTTPRequestHandler.html_filepath = self.html_filepath

        # SSL Context Setup
        context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        context.load_cert_chain(certfile=self.cert_file, keyfile=self.key_file)

        self.httpd = SecureHTTPServer(
            ('0.0.0.0', self.server_port),
            MobileSensorHTTPRequestHandler,
            context
        )

        # Start Threaded HTTPS Server
        self.server_thread = threading.Thread(target=self.httpd.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.get_logger().info('HTTPS server started. Connect via one of the URLs below in your smartphone browser:')
        for ip in self.ip_addresses:
            self.get_logger().info(f'  - https://{ip}:{self.server_port}')

    def get_all_local_ips(self) -> List[str]:
        """Discovers all available IPv4 addresses on the host system."""
        ips = []
        try:
            # Method 1: Hostname Resolution
            hostname = socket.gethostname()
            addr_info = socket.getaddrinfo(hostname, None, socket.AF_INET)
            for item in addr_info:
                ip = item[4][0]
                if ip not in ips and not ip.startswith('127.'):
                    ips.append(ip)
        except Exception:
            pass

        try:
            # Method 2: UDP Socket Probe
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('10.255.255.255', 1))
            ip = s.getsockname()[0]
            if ip not in ips and not ip.startswith('127.'):
                ips.append(ip)
            s.close()
        except Exception:
            pass

        if not ips:
            ips = ['127.0.0.1']

        return ips

    def parse_stamp(self, client_ts_ms_str: str):
        """Parses client timestamp in milliseconds string to builtin_interfaces.msg.Time."""
        if client_ts_ms_str:
            try:
                client_ts_ms = float(client_ts_ms_str)
                client_ts_ns = int(client_ts_ms * 1e6)
                sec = client_ts_ns // 1_000_000_000
                nanosec = client_ts_ns % 1_000_000_000
                from builtin_interfaces.msg import Time
                stamp = Time()
                stamp.sec = sec
                stamp.nanosec = nanosec
                return stamp
            except Exception:
                pass
        return self.get_clock().now().to_msg()

    def generate_certificates(self) -> None:
        needs_regen = not os.path.exists(self.cert_file) or not os.path.exists(self.key_file)
        if not needs_regen:
            try:
                result = subprocess.run(
                    ['openssl', 'x509', '-in', self.cert_file, '-text', '-noout'],
                    capture_output=True, text=True
                )
                for ip in self.ip_addresses:
                    if f'IP Address:{ip}' not in result.stdout:
                        needs_regen = True
                        break
            except Exception:
                needs_regen = True

        if needs_regen:
            self.get_logger().info('Generating self-signed SSL certificate with multi-IP SAN...')
            san_list = [f'IP:{ip}' for ip in self.ip_addresses] + ['IP:127.0.0.1', 'DNS:localhost']
            san_str = ','.join(san_list)

            primary_ip = self.ip_addresses[0] if self.ip_addresses else '127.0.0.1'
            cmd = [
                'openssl', 'req', '-newkey', 'rsa:2048', '-nodes',
                '-keyout', self.key_file, '-x509', '-days', '365',
                '-out', self.cert_file,
                '-subj', f'/C=KR/ST=Seoul/L=Seoul/O=WROS/CN={primary_ip}',
                '-addext', f'subjectAltName={san_str}',
                '-addext', 'extendedKeyUsage=serverAuth',
                '-addext', 'keyUsage=digitalSignature,keyEncipherment',
            ]
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if result.returncode != 0:
                self.get_logger().error(f'Certificate generation failed: {result.stderr.decode()}')
            else:
                self.get_logger().info('Multi-IP Certificate generated successfully.')

    def _push_to_queue(self, target_queue: queue.Queue, item) -> None:
        """Atomically pushes item to bounded queue without ever blocking HTTP handler thread."""
        while True:
            try:
                target_queue.put_nowait(item)
                return
            except queue.Full:
                try:
                    target_queue.get_nowait()
                except queue.Empty:
                    pass

    def enqueue_device_info(self, data: dict) -> None:
        self._push_to_queue(self.sensor_publish_queue, (self._publish_device_info, (data,)))

    def enqueue_imu(self, data: dict, stamp) -> None:
        self.last_data_received_time = time.time()
        self._push_to_queue(self.sensor_publish_queue, (self.imu_bridge.handle_imu, (data, stamp)))

    def enqueue_battery(self, data: dict, stamp) -> None:
        self.last_data_received_time = time.time()
        self._push_to_queue(self.sensor_publish_queue, (self.battery_bridge.handle_battery, (data, stamp)))

    def enqueue_gps(self, data: dict, stamp) -> None:
        self.last_data_received_time = time.time()
        self._push_to_queue(self.sensor_publish_queue, (self.gps_bridge.handle_gps, (data, stamp)))

    def enqueue_upload(self, post_data: bytes, content_type: str, stamp) -> None:
        self.last_data_received_time = time.time()
        self._push_to_queue(self.camera_publish_queue, (self.camera_bridge.handle_upload, (post_data, content_type, stamp)))

    def _check_heartbeat_timeout(self) -> None:
        """Automatically stops rosbag2 recording if client stream disconnects or stops sending data."""
        if self.record_process is not None and self.last_data_received_time > 0.0:
            if time.time() - self.last_data_received_time > self.heartbeat_timeout:
                self.get_logger().info(f'No incoming sensor data for {self.heartbeat_timeout} seconds. Auto-closing rosbag2 recording...')
                self.stop_bag_recording()

    def _publish_device_info(self, data: dict) -> None:
        model = data.get('model', 'Unknown Device')
        os_info = data.get('os', 'Unknown OS')
        browser = data.get('browser', 'Unknown Browser')
        camera_label = data.get('cameraLabel', 'Unknown Camera')
        width = data.get('width', 0)
        height = data.get('height', 0)
        fps = data.get('frameRate', 0)

        log_msg = (
            f"Connected Device -> Model: {model} | OS: {os_info} | Browser: {browser} | "
            f"Camera: {camera_label} ({width}x{height} @ {fps}fps)"
        )
        self.get_logger().info(log_msg)
        if self.device_info_publisher is not None:
            info_msg = String()
            info_msg.data = json.dumps(data)
            self.device_info_publisher.publish(info_msg)

    def _process_publish_queues(self) -> None:
        # Process Sensor Queue
        while not self.sensor_publish_queue.empty():
            try:
                func, args = self.sensor_publish_queue.get_nowait()
                func(*args)
            except queue.Empty:
                break
            except Exception as e:
                self.get_logger().error(f"Error processing sensor publish queue: {e}")

        # Process Camera Queue
        while not self.camera_publish_queue.empty():
            try:
                func, args = self.camera_publish_queue.get_nowait()
                func(*args)
            except queue.Empty:
                break
            except Exception as e:
                self.get_logger().error(f"Error processing camera publish queue: {e}")

    def start_bag_recording(self) -> None:
        """Starts automatic rosbag2 recording subprocess with QoS overrides for Best Effort sensor topics."""
        if self.record_process is not None:
            return
        self.last_data_received_time = time.time()
        import datetime
        timestamp_str = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        output_path = os.path.join(self.bag_dir, f'rosbag2_{timestamp_str}')

        qos_override_file = os.path.join(self.bag_dir, 'qos_overrides.yaml')
        qos_content = (
            "/image_raw/compressed:\n"
            "  reliability: best_effort\n"
            "  durability: volatile\n"
            "  history: keep_last\n"
            "  depth: 10\n"
            "/imu/data_raw:\n"
            "  reliability: best_effort\n"
            "  durability: volatile\n"
            "  history: keep_last\n"
            "  depth: 10\n"
            "/robot/battery:\n"
            "  reliability: best_effort\n"
            "  durability: volatile\n"
            "  history: keep_last\n"
            "  depth: 10\n"
            "/robot/gps:\n"
            "  reliability: best_effort\n"
            "  durability: volatile\n"
            "  history: keep_last\n"
            "  depth: 10\n"
            "/camera_info:\n"
            "  reliability: best_effort\n"
            "  durability: volatile\n"
            "  history: keep_last\n"
            "  depth: 10\n"
        )
        with open(qos_override_file, 'w') as f:
            f.write(qos_content)

        cmd = [
            'ros2', 'bag', 'record',
            '-o', output_path,
            '--qos-profile-overrides-path', qos_override_file,
            '--topics',
            '/image_raw/compressed',
            '/imu/data_raw',
            '/robot/battery',
            '/robot/gps',
            '/mobile_sensor_bridge/device_info',
            '/camera_info'
        ]
        try:
            self.record_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
            self.get_logger().info(f'Started automatic rosbag2 recording: {output_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to start rosbag2 recording: {e}')

    def stop_bag_recording(self) -> None:
        """Stops active rosbag2 recording process group safely using SIGINT for DB flush."""
        if self.record_process is None:
            return
        try:
            import signal
            pgid = os.getpgid(self.record_process.pid)
            os.killpg(pgid, signal.SIGINT)
            self.record_process.wait(timeout=5)
            self.get_logger().info('Stopped automatic rosbag2 recording successfully.')
        except Exception as e:
            self.get_logger().warn(f'rosbag2 process cleanup warning: {e}')
            try:
                import signal
                os.killpg(os.getpgid(self.record_process.pid), signal.SIGKILL)
            except Exception:
                pass
        finally:
            self.record_process = None

    def destroy_node(self) -> None:
        self.stop_bag_recording()
        try:
            self.httpd.server_close()
            self.httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MobileSensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
