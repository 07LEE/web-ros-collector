import rclpy
from rclpy.node import Node

# 1. Node 클래스를 상속받아 나만의 노드 정의
class MyFirstNode(Node):
    def __init__(self):
        # 노드 이름을 'my_first_node'로 초기화
        super().__init__('my_first_node')
        # 터미널에 로그 출력
        self.get_logger().info('Hello ROS 2 Node!')

def main(args=None):
    # 2. ROS 2 통신 라이브러리 초기화
    rclpy.init(args=args)
    
    # 3. 노드 객체 생성
    node = MyFirstNode()
    
    # 4. 노드가 종료될 때까지 이벤트 루프 유지 (대기 상태)
    rclpy.spin(node)
    
    # 5. 노드 종료 및 자원 해제
    rclpy.shutdown()

if __name__ == '__main__':
    main()