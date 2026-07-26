"""ROS2 Main Node for Mobile Sensor Bridge, integrating CameraBridge and ImuBridge."""

import http.server
import json
import os
import socket
import socketserver
import ssl
import subprocess
import threading
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mobile_sensor_bridge.camera_bridge import CameraBridge
from mobile_sensor_bridge.imu_bridge import ImuBridge


class MobileSensorHTTPRequestHandler(http.server.BaseHTTPRequestHandler):
    """HTTPS Request Handler routing web requests to CameraBridge and ImuBridge handlers."""

    node = None
    html_filepath = ""

    def log_message(self, format, *args):
        """Suppresses default HTTP request logging to prevent terminal output spam."""
        pass

    def do_GET(self):
        """Handles HTTP GET requests to serve static web assets."""
        filename = 'index.html' if self.path in ('/', '/index.html') else self.path.lstrip('/')
        web_dir = os.path.dirname(self.html_filepath)
        target_path = os.path.abspath(os.path.join(web_dir, filename))

        if not target_path.startswith(web_dir):
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

    def do_POST(self):
        """Handles HTTP POST requests and routes payloads to CameraBridge or ImuBridge."""
        if self.path == '/device_info':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                data = json.loads(post_data.decode('utf-8'))

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

                if self.node is not None:
                    self.node.get_logger().info(log_msg)
                    if self.node.device_info_publisher is not None:
                        info_msg = String()
                        info_msg.data = json.dumps(data)
                        self.node.device_info_publisher.publish(info_msg)

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

                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b"OK")

                if self.node is not None and self.node.imu_bridge is not None:
                    stamp = self.node.get_clock().now().to_msg()
                    self.node.imu_bridge.handle_imu(data, stamp)

            except Exception:
                self.send_response(500)
                self.end_headers()

        elif self.path == '/upload':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)
                content_type = self.headers.get('Content-Type', '')

                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Connection', 'keep-alive')
                self.end_headers()
                self.wfile.write(b"OK")

                if self.node is not None and self.node.camera_bridge is not None:
                    stamp = self.node.get_clock().now().to_msg()
                    self.node.camera_bridge.handle_upload(post_data, content_type, stamp)

            except Exception:
                self.send_response(500)
                self.end_headers()


class SecureHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    """Multi-threaded HTTPS server wrapping client sockets individually per connection."""

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

        # Instantiate Camera and IMU Bridges
        self.camera_bridge = CameraBridge(self)
        self.imu_bridge = ImuBridge(self)

        # Device Info Publisher
        self.device_info_publisher = self.create_publisher(
            String,
            '/mobile_sensor_bridge/device_info',
            10
        )

        self.ip_addresses = self.get_all_local_ips()

        # SSL Certificates Setup
        self.cert_dir = os.path.join(os.getcwd(), 'certs')
        os.makedirs(self.cert_dir, exist_ok=True)
        self.cert_file = os.path.join(self.cert_dir, 'cert.pem')
        self.key_file = os.path.join(self.cert_dir, 'key.pem')

        self.generate_certificates()

        # Web Assets Location Resolution
        self.html_filepath = os.path.join(
            os.getcwd(),
            'mobile_sensor_bridge',
            'web',
            'index.html'
        )
        if not os.path.exists(self.html_filepath):
            self.html_filepath = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'share',
                'mobile_sensor_bridge',
                'web',
                'index.html'
            )

        self.server_port = 8443

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

    def destroy_node(self) -> None:
        self.httpd.shutdown()
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
