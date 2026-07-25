import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import ssl
import http.server
import threading
import os
import subprocess
import socket

class CameraHTTPRequestHandler(http.server.BaseHTTPRequestHandler):
    node = None
    publisher = None
    html_filepath = ""

    def log_message(self, format, *args):
        # Prevent spamming HTTP request logs in terminal
        pass

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            try:
                with open(self.html_filepath, 'rb') as f:
                    content = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
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
        if self.path == '/upload':
            try:
                content_length = int(self.headers['Content-Length'])
                post_data = self.rfile.read(content_length)

                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b"OK")

                if self.publisher is not None and self.node is not None:
                    msg = CompressedImage()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.header.frame_id = 'phone_camera'
                    msg.format = 'jpeg'
                    msg.data = post_data
                    self.publisher.publish(msg)
            except Exception as e:
                self.send_response(500)
                self.end_headers()

class CameraBridgeNode(Node):
    def __init__(self):
        super().__init__('camera_bridge_node')
        self.publisher = self.create_publisher(
            CompressedImage,
            '/image_raw/compressed',
            10
        )

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
        self.ip_address = self.get_local_ip()
        
        CameraHTTPRequestHandler.node = self
        CameraHTTPRequestHandler.publisher = self.publisher
        CameraHTTPRequestHandler.html_filepath = self.html_filepath

        self.httpd = http.server.ThreadingHTTPServer(
            ('0.0.0.0', self.server_port),
            CameraHTTPRequestHandler
        )

        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(certfile=self.cert_file, keyfile=self.key_file)
        self.httpd.socket = context.wrap_socket(
            self.httpd.socket,
            server_side=True
        )

        self.server_thread = threading.Thread(target=self.httpd.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.get_logger().info(f'HTTPS 서버 시작: https://{self.ip_address}:{self.server_port}')
        self.get_logger().info('스마트폰 브라우저로 위 주소에 접속하여 주십시오.')

    def get_local_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('10.255.255.255', 1))
            ip = s.getsockname()[0]
        except Exception:
            ip = '127.0.0.1'
        finally:
            s.close()
        return ip

    def generate_certificates(self):
        if not os.path.exists(self.cert_file) or not os.path.exists(self.key_file):
            self.get_logger().info('자체 서명 SSL 인증서를 생성하는 중입니다...')
            cmd = [
                'openssl', 'req', '-newkey', 'rsa:2048', '-nodes',
                '-keyout', self.key_file, '-x509', '-days', '365',
                '-out', self.cert_file,
                '-subj', '/C=KR/ST=Seoul/L=Seoul/O=WROS/CN=localhost'
            ]
            subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.get_logger().info('인증서 생성이 완료되었습니다.')

    def destroy_node(self):
        self.httpd.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
