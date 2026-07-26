"""ROS2 node for bridging smartphone web camera feed into ROS2 Image and CompressedImage topics."""

import http.server
import json
import os
import socket
import socketserver
import ssl
import subprocess
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class CameraHTTPRequestHandler(http.server.BaseHTTPRequestHandler):
    """HTTP request handler for receiving JPEG frames and device info from smartphone browser."""

    node = None
    compressed_publisher = None
    raw_publisher = None
    device_info_publisher = None
    html_filepath = ""

    def log_message(self, format, *args):
        """Suppresses default HTTP request logging to prevent terminal output spam."""
        pass

    def do_GET(self):
        """Handles HTTP GET requests to serve static files from the web directory."""
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
        """Handles HTTP POST requests to receive JPEG binary data or device information."""
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
                    if self.device_info_publisher is not None:
                        info_msg = String()
                        info_msg.data = json.dumps(data)
                        self.device_info_publisher.publish(info_msg)

                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b"OK")
            except Exception as e:
                self.send_response(500)
                self.end_headers()

        elif self.path == '/upload':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)

                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Connection', 'keep-alive')
                self.end_headers()
                self.wfile.write(b"OK")

                if self.node is not None:
                    now = self.node.get_clock().now().to_msg()

                    # 1. Publish CompressedImage topic (/image_raw/compressed)
                    if self.compressed_publisher is not None:
                        img_format = 'png' if 'png' in self.headers.get('Content-Type', '') else 'jpeg'
                        comp_msg = CompressedImage()
                        comp_msg.header.stamp = now
                        comp_msg.header.frame_id = 'phone_camera'
                        comp_msg.format = img_format
                        comp_msg.data = post_data
                        self.compressed_publisher.publish(comp_msg)

                    # 2. Lazy decoding: publish raw Image topic (/image_raw) ONLY if there are active subscribers
                    if self.raw_publisher is not None and self.raw_publisher.get_subscription_count() > 0:
                        np_arr = np.frombuffer(post_data, np.uint8)
                        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        if cv_img is not None:
                            height, width, channels = cv_img.shape
                            raw_msg = Image()
                            raw_msg.header.stamp = now
                            raw_msg.header.frame_id = 'phone_camera'
                            raw_msg.height = height
                            raw_msg.width = width
                            raw_msg.encoding = 'bgr8'
                            raw_msg.is_bigendian = 0
                            raw_msg.step = width * channels
                            raw_msg.data = cv_img.tobytes()
                            self.raw_publisher.publish(raw_msg)

            except Exception as e:
                self.send_response(500)
                self.end_headers()


class SecureHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    """Multi-threaded HTTPS server wrapping client sockets individually per connection."""

    def __init__(self, server_address, request_handler_class, ssl_context):
        """Initializes the HTTPS server with SSL context.

        Args:
            server_address (tuple): Binding host and port tuple.
            request_handler_class (type): Request handler class.
            ssl_context (ssl.SSLContext): SSL context configured for server authentication.
        """
        super().__init__(server_address, request_handler_class)
        self.ssl_context = ssl_context

    def get_request(self):
        """Accepts an incoming connection and wraps the socket with SSL context.

        Returns:
            tuple: (ssl_socket, client_address)
        """
        newsock, fromaddr = self.socket.accept()
        conn = self.ssl_context.wrap_socket(newsock, server_side=True)
        return conn, fromaddr


class CameraBridgeNode(Node):
    """ROS2 Node managing the HTTPS server and publishing Image & CompressedImage messages."""

    def __init__(self):
        """Initializes the CameraBridgeNode, generates SSL certificates, and starts HTTPS server."""
        super().__init__('camera_bridge_node')

        self.compressed_publisher = self.create_publisher(
            CompressedImage,
            '/image_raw/compressed',
            10
        )
        self.raw_publisher = self.create_publisher(
            Image,
            '/image_raw',
            10
        )
        self.device_info_publisher = self.create_publisher(
            String,
            '/camera_bridge/device_info',
            10
        )

        self.ip_address = self.get_local_ip()

        self.cert_dir = os.path.join(os.getcwd(), 'certs')
        os.makedirs(self.cert_dir, exist_ok=True)
        self.cert_file = os.path.join(self.cert_dir, 'cert.pem')
        self.key_file = os.path.join(self.cert_dir, 'key.pem')

        self.generate_certificates()

        self.html_filepath = os.path.join(
            os.getcwd(),
            'camera_bridge',
            'web',
            'index.html'
        )
        if not os.path.exists(self.html_filepath):
            self.html_filepath = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'share',
                'camera_bridge',
                'web',
                'index.html'
            )

        self.server_port = 8443

        CameraHTTPRequestHandler.node = self
        CameraHTTPRequestHandler.compressed_publisher = self.compressed_publisher
        CameraHTTPRequestHandler.raw_publisher = self.raw_publisher
        CameraHTTPRequestHandler.device_info_publisher = self.device_info_publisher
        CameraHTTPRequestHandler.html_filepath = self.html_filepath

        context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        context.load_cert_chain(certfile=self.cert_file, keyfile=self.key_file)

        self.httpd = SecureHTTPServer(
            ('0.0.0.0', self.server_port),
            CameraHTTPRequestHandler,
            context
        )

        self.server_thread = threading.Thread(target=self.httpd.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.get_logger().info(f'HTTPS server started: https://{self.ip_address}:{self.server_port}')
        self.get_logger().info('Open the URL above in your smartphone browser.')

    def get_local_ip(self) -> str:
        """Determines the primary local IPv4 address of the system.

        Returns:
            str: Local IPv4 address.
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('10.255.255.255', 1))
            ip = s.getsockname()[0]
        except Exception:
            ip = '127.0.0.1'
        finally:
            s.close()
        return ip

    def generate_certificates(self) -> None:
        """Generates self-signed SSL certificate with IP SAN and serverAuth EKU."""
        needs_regen = not os.path.exists(self.cert_file) or not os.path.exists(self.key_file)
        if not needs_regen:
            try:
                result = subprocess.run(
                    ['openssl', 'x509', '-in', self.cert_file, '-text', '-noout'],
                    capture_output=True, text=True
                )
                if f'IP Address:{self.ip_address}' not in result.stdout:
                    needs_regen = True
            except Exception:
                needs_regen = True

        if needs_regen:
            self.get_logger().info('Generating self-signed SSL certificate...')
            san = f'IP:{self.ip_address},IP:127.0.0.1,DNS:localhost'
            cmd = [
                'openssl', 'req', '-newkey', 'rsa:2048', '-nodes',
                '-keyout', self.key_file, '-x509', '-days', '365',
                '-out', self.cert_file,
                '-subj', f'/C=KR/ST=Seoul/L=Seoul/O=WROS/CN={self.ip_address}',
                '-addext', f'subjectAltName={san}',
                '-addext', 'extendedKeyUsage=serverAuth',
                '-addext', 'keyUsage=digitalSignature,keyEncipherment',
            ]
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if result.returncode != 0:
                self.get_logger().error(
                    f'Certificate generation failed: {result.stderr.decode()}'
                )
            else:
                self.get_logger().info('Certificate generated successfully.')

    def destroy_node(self) -> None:
        """Shuts down the HTTP server and destroys the ROS2 node."""
        self.httpd.shutdown()
        super().destroy_node()


def main(args=None):
    """Main entry point for starting the camera bridge node."""
    rclpy.init(args=args)
    node = CameraBridgeNode()
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
