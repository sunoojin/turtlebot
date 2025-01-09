import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .yolo_detection import YoloDetector
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # 로봇 제어 관련 초기화
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.command = Twist()

        # YOLO 물체 감지 객체 초기화
        self.yolo_detector = YoloDetector()

    def timer_callback(self):
        self.publisher.publish(self.command)
        # self.get_logger().info(f'현재 속도 명령: {self.command}')

    def move_forward(self, linear_speed=0.2, duration=2.4):   
        if linear_speed < 0:
            raise ValueError('회전 속도는 양수이어야 합니다.')
        self.command.linear.x = linear_speed
        self.command.angular.z = 0.0
        self._publish_for_duration(duration)
    
    def move_forward_diagonal(self, linear_speed=0.2, duration=3.4): #루트2 이동: 실험 필요
        if linear_speed < 0:
            raise ValueError('회전 속도는 양수이어야 합니다.')
        self.command.linear.x = linear_speed
        self.command.angular.z = 0.0
        self._publish_for_duration(duration)    
        
    def move_backward(self, linear_speed=0.2, duration=2.3):
        if linear_speed < 0:
            raise ValueError('이동 속도는 양수이어야 합니다.')
        self.command.linear.x = -linear_speed
        self.command.angular.z = 0.0
        self._publish_for_duration(duration)    
    
    def rotate_left(self, angular_speed=0.2, duration=7.8):
        if angular_speed < 0:
            raise ValueError('회전 속도는 양수이어야 합니다.')
        self.command.linear.x = 0.0
        self.command.angular.z = angular_speed
        self._publish_for_duration(duration)

    def rotate_right(self, angular_speed=0.2, duration=8.2):
        if angular_speed < 0:
            raise ValueError('회전 속도는 양수이어야 합니다.')
        self.command.linear.x = 0.0
        self.command.angular.z = -angular_speed
        self._publish_for_duration(duration)

    def rotate_left_diagonal(self, angular_speed=0.2, duration=4.7): #대각선: 실험 필요
        if angular_speed < 0:
            raise ValueError('회전 속도는 양수이어야 합니다.')
        self.command.linear.x = 0.0
        self.command.angular.z = angular_speed
        self._publish_for_duration(duration)
    
    def rotate_right_diagonal(self, angular_speed=0.2, duration=4.5): #대각선: 실험 필요
        if angular_speed < 0:
            raise ValueError('회전 속도는 양수이어야 합니다.')
        self.command.linear.x = 0.0
        self.command.angular.z = -angular_speed
        self._publish_for_duration(duration)
        
    def stop(self, duration=1.0):
        self.command.linear.x = 0.0
        self.command.angular.z = 0.0
        self._publish_for_duration(duration)

    def _publish_for_duration(self, duration):
        end_time = time.time() + duration
        while rclpy.ok() and time.time() < end_time:
            self.publisher.publish(self.command)
            time.sleep(0.1)

    def start_detection(self, duration, target_class='bottle'):
        self.yolo_detector.detect(duration, target_class)
        while not self.yolo_detector.detection_complete and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        # 탐지 완료 후 결과 반환
        if self.yolo_detector.detection_complete and self.yolo_detector.target_class in self.yolo_detector.detected_objects:
            return True     # 감지 성공
        return False        # 감지 실패

    #이동 함수    
    def move_1X0(self):
        self.get_logger().info('move_1X0')
        self.move_forward()
        self.stop()
                
    def move_2X0(self):
        self.get_logger().info('move_2X0')
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_3X0(self):
        self.get_logger().info('move_3X0')
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_4X0(self):
        self.get_logger().info('move_4X0')
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_5X0(self):
        self.get_logger().info('move_5X0')
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_6X0(self):
        self.get_logger().info('move_6X0')
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_7X0(self):
        self.get_logger().info('move_7X0')
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()
    
    def move_0X1(self):
        self.get_logger().info('move_0X1')
        self.rotate_right()
        self.move_forward()
        self.rotate_left()
        self.stop()
    
    def move_1X1(self):
        self.get_logger().info('move_1X1')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.stop()

    def move_2X1(self):
        self.get_logger().info('move_2X1')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.stop()
    
    def move_3X1(self):
        self.get_logger().info('move_3X1')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_4X1(self):
        self.get_logger().info('move_4X1')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_5X1(self):
        self.get_logger().info('move_5X1')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_6X1(self):
        self.get_logger().info('move_6X1')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_7X1(self):
        self.get_logger().info('move_7X1')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_0X2(self):
        self.get_logger().info('move_0X2')
        self.rotate_right()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()
    
    def move_1X2(self):
        self.get_logger().info('move_1X2')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.rotate_left()
        self.stop()
    
    def move_2X2(self):
        self.get_logger().info('move_2X2')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.stop()

    def move_3X2(self):
        self.get_logger().info('move_3X2')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.stop()

    def move_4X2(self):
        self.get_logger().info('move_4X2')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_5X2(self):
        self.get_logger().info('move_5X2')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_6X2(self):
        self.get_logger().info('move_6X2')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_7X2(self):
        self.get_logger().info('move_7X2')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_0X3(self):
        self.get_logger().info('move_0X3')
        self.rotate_right()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()
    
    def move_1X3(self):
        self.get_logger().info('move_1X3')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_2X3(self):
        self.get_logger().info('move_2X3')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.rotate_left()
        self.stop()
    
    def move_3X3(self):
        self.get_logger().info('move_3X3')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.stop()

    def move_4X3(self):
        self.get_logger().info('move_4X3')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.stop()

    def move_5X3(self):
        self.get_logger().info('move_5X3')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_6X3(self):
        self.get_logger().info('move_6X3')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_7X3(self):
        self.get_logger().info('move_7X3')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_0X4(self):
        self.get_logger().info('move_0X4')
        self.rotate_right()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_1X4(self):
        self.get_logger().info('move_1X4')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_2X4(self):
        self.get_logger().info('move_2X4')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_3X4(self):
        self.get_logger().info('move_3X4')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_4X4(self):
        self.get_logger().info('move_4X4')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.stop()

    def move_5X4(self):
        self.get_logger().info('move_5X4')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.stop()

    def move_6X4(self):
        self.get_logger().info('move_6X4')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_7X4(self):
        self.get_logger().info('move_7X4')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_0X5(self):
        self.get_logger().info('move_0X5')
        self.rotate_right()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_1X5(self):
        self.get_logger().info('move_1X5')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_2X5(self):
        self.get_logger().info('move_2X5')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_3X5(self):
        self.get_logger().info('move_3X5')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_4X5(self):
        self.get_logger().info('move_4X5')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_5X5(self):
        self.get_logger().info('move_5X5')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.stop()

    def move_6X5(self):
        self.get_logger().info('move_6X5')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.stop()

    def move_7X5(self):
        self.get_logger().info('move_7X5')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.move_forward()
        self.stop()

    def move_0X6(self):
        self.get_logger().info('move_0X6')
        self.rotate_right()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_1X6(self):
        self.get_logger().info('move_1X6')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_2X6(self):
        self.get_logger().info('move_2X6')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_3X6(self):
        self.get_logger().info('move_3X6')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_4X6(self):
        self.get_logger().info('move_4X6')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_5X6(self):
        self.get_logger().info('move_5X6')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_6X6(self):
        self.get_logger().info('move_6X6')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.stop()

    def move_7X6(self):
        self.get_logger().info('move_7X6')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_left_diagonal()
        self.move_forward()
        self.stop()

    def move_0X7(self):
        self.get_logger().info('move_0X7')
        self.rotate_right()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_1X7(self):
        self.get_logger().info('move_1X7')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_2X7(self):
        self.get_logger().info('move_2X7')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_3X7(self):
        self.get_logger().info('move_3X7')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_4X7(self):
        self.get_logger().info('move_4X7')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_5X7(self):
        self.get_logger().info('move_5X7')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.move_forward()
        self.rotate_left()
        self.stop()

    def move_6X7(self):
        self.get_logger().info('move_6X7')
        self.rotate_right_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.move_forward_diagonal()
        self.rotate_right_diagonal()
        self.move_forward()
        self.rotate_left()
        self.stop()

    #사진 위치로 이동 + 사진 scan 함수
    def detection(self, m, n, k):
        """매개 변수: 상자 좌표 = (m, n), 사진 위치 = k"""
        self.get_logger().info('detection 함수 시작')
        if m == 0:
            if k == 1:  # 우회전-정지-[SCAN]
                self.get_logger().info('m=0, k=1')
                self.rotate_right()
                self.stop()
                self.start_detection(5.0)  # 예시로 5초 동안 탐지 실행 (필요에 따라 조정)

            if k == 3:  # 직진-우회전-직진-직진-우회전-직진-우회전-[정지]-[SCAN]
                self.get_logger().info('m=0, k=3')
                self.move_forward()
                self.rotate_right()
                self.move_forward()
                self.move_forward()
                self.rotate_right()
                self.move_forward()
                self.rotate_right()
                self.stop()
                self.start_detection(5.0)

            if k == 4:  # 직진-우회전-직진-우회전-[정지]-[SCAN]
                self.get_logger().info('m=0, k=4')
                self.move_forward()
                self.rotate_right()
                self.move_forward()
                self.rotate_right()
                self.stop()
                self.start_detection(5.0)

        if n == 0:
            if k == 2:  # [정지]-[SCAN]
                self.get_logger().info('n=0, k=2')
                self.stop()
                self.start_detection(5.0)

            if k == 3:  # 우회전-직진-좌회전-직진-좌회전-[정지]-[SCAN]
                self.get_logger().info('n=0, k=3')
                self.rotate_right()
                self.move_forward()
                self.rotate_left()
                self.move_forward()
                self.rotate_left()
                self.stop()
                self.start_detection(5.0)

            if k == 4:  # 우회전-직진-좌회전-직진-직진-좌회전-직진-좌회전-[정지]-[SCAN]
                self.get_logger().info('n=0, k=4')
                self.rotate_right()
                self.move_forward()
                self.rotate_left()
                self.move_forward()
                self.move_forward()
                self.rotate_left()
                self.move_forward()
                self.rotate_left()
                self.stop()
                self.start_detection(5.0)

        else:
            if k == 1:  # 직진-우회전-[정지]-[SCAN]
                self.get_logger().info('else, k=1')
                self.move_forward()
                self.rotate_right()
                self.stop()
                self.start_detection(5.0)

            if k == 2:  # 우회전-직진-좌회전-[정지]-[SCAN]
                self.get_logger().info('else, k=2')
                self.rotate_right()
                self.move_forward()
                self.rotate_left()
                self.stop()
                self.start_detection(5.0)

            if k == 3:  # 우회전-직진-직진-좌회전-직진-좌회전-[정지]-[SCAN]
                self.get_logger().info('else, k=3')
                self.rotate_right()
                self.move_forward()
                self.move_forward()
                self.rotate_left()
                self.move_forward()
                self.rotate_left()
                self.stop()
                self.start_detection(5.0)

            if k == 4:  # 직진-직진-우회전-직진-우회전-[정지]-[SCAN]
                self.get_logger().info('else, k=4')
                self.move_forward()
                self.move_forward()
                self.rotate_right()
                self.move_forward()
                self.rotate_right()
                self.stop()
                self.start_detection(5.0)
        self.get_logger().info('detection 함수 완료')

    def escape(self, m1, n1, k, m2, n2):
        """매개변수: 현재 목표 상자 좌표: (m1, n1), 다음 목표 상자 좌표: (m2, n2)
        찍은 사진 위치 = k"""
        self.get_logger().info('escape 함수 시작')        
        if n1 == n2:
            if m1 < m2:
                if k == 1:  # 좌회전-직진-[정지]
                    self.get_logger().info('n1=n2, m1<m2, k=1')
                    self.rotate_left()
                    self.move_forward()
                    self.stop()

                if k == 2:  # 좌회전-직진-우회전-직진-직진-[정지]
                    self.get_logger().info('n1=n2, m1<m2, k=2')
                    self.rotate_left()
                    self.move_forward()
                    self.rotate_right()
                    self.move_forward()
                    self.move_forward()
                    self.stop()

                if k == 3:  # 좌회전-직진-우회전-직진-직진-우회전-직진-직진-[정지]
                    self.get_logger().info('n1=n2, m1<m2, k=3')
                    self.rotate_left()
                    self.move_forward()
                    self.rotate_right()
                    self.move_forward()
                    self.move_forward()
                    self.rotate_right()
                    self.move_forward()
                    self.move_forward()
                    self.stop()

                if k == 4:  # 우회전-직진-우회전-[정지]
                    self.get_logger().info('n1=n2, m1<m2, k=4')
                    self.rotate_right()
                    self.move_forward()
                    self.rotate_right()
                    self.stop()

            if m1 > m2:  #예외사항
                if k == 1:  # 우회전-직진-[정지]
                    self.get_logger().info('n1=n2, m1>m2, k=1')
                    self.rotate_right()
                    self.move_forward()
                    self.stop()

                if k == 2:  # 좌회전-직진-좌회전-[정지]
                    self.get_logger().info('n1=n2, m1>m2, k=2')
                    self.rotate_left()
                    self.move_forward()
                    self.rotate_left()
                    self.stop()

                if k == 3:  # 좌회전-직진-우회전-직진-직진-좌회전-[정지]
                    self.get_logger().info('n1=n2, m1>m2, k=3')
                    self.rotate_left()
                    self.move_forward()
                    self.rotate_right()
                    self.move_forward()
                    self.move_forward()
                    self.rotate_left()
                    self.stop()

                if k == 4:  # 우회전-직진-좌회전-직진-직진-[정지]
                    self.get_logger().info('n1=n2, m1>m2, k=4')
                    self.rotate_right()
                    self.move_forward()
                    self.rotate_left()
                    self.move_forward()
                    self.move_forward()
                    self.stop()

        elif m1==m2:
            if n1<n2:
                if m1==0:
                    if k==1: #좌회전-직진-우회전-직진-[정지]
                        self.get_logger().info('m1=m2, n1<n2, else, k=1')
                        self.rotate_left()
                        self.move_forward()
                        self.rotate_right()
                        self.move_forward()
                        self.stop()                
                    if k==2: #좌회전-직진-우회전-직진-직진-우회전-직진-[정지]
                        self.get_logger().info('m1=m2, n1<n2, else, k=2')
                        self.rotate_left()
                        self.move_forward()
                        self.rotate_right()
                        self.move_forward()
                        self.move_forward()
                        self.rotate_right()
                        self.move_forward()
                        self.stop()
                    if k==3: #좌회전-직진-우회전-직진-직진-우회전-직진-직진-우회전-직진-[정지]
                        self.get_logger().info('m1=m2, n1<n2, else, k=3')
                        self.rotate_left()
                        self.move_forward()
                        self.rotate_right()
                        self.move_forward()
                        self.move_forward()
                        self.rotate_right()
                        self.move_forward()
                        self.move_forward()
                        self.rotate_right()
                        self.move_forward()
                        self.stop()
                    if k==4: #좌회전-[정지]
                        self.get_logger().info('m1=m2, n1<n2, else, k=4')
                        self.rotate_left()
                        self.stop()
                      
                else:
                    if k==1: #우회전-직진-좌회전-직진-직진-[정지]
                        self.get_logger().info('m1=m2, n1<n2, m1=7, k=1')
                        self.rotate_right()
                        self.move_forward()
                        self.rotate_left()
                        self.move_forward()
                        self.move_forward()
                        self.stop()   
                    if k==2: #우회전-직진-[정지]
                        self.get_logger().info('m1=m2, n1<n2, m1=7, k=2')
                        self.rotate_right()
                        self.move_forward()
                        self.stop()        
                    if k==3: #좌회전-직진-좌회전-[정지]
                        self.get_logger().info('m1=m2, n1<n2, m1=7, k=3')
                        self.rotate_left()
                        self.move_forward()
                        self.rotate_left()
                        self.stop()            
                    if k==4: #우회전-직진-좌회전-직진-직진-좌회전-직진-직진-[정지]
                        self.get_logger().info('m1=m2, n1<n2, m1=7, k=4')
                        self.rotate_right() 
                        self.move_forward()
                        self.rotate_left()
                        self.move_forward()
                        self.move_forward()
                        self.rotate_left()
                        self.move_forward()
                        self.move_forward()
                        self.stop()
                    

        elif m1<m2 and n1<n2:
            if k==1: #좌회전-직진-우회전-직진-직진-좌회전-[정지]
                self.get_logger().info('m1<m2 and n1<n2, k=1')
                self.rotate_left()
                self.move_forward()
                self.rotate_right()
                self.move_forward()
                self.move_forward()
                self.rotate_left()
                self.stop()
            if k==2: #우회전-직진-좌회전-직진-직진-[정지]
                self.get_logger().info('m1<m2 and n1<n2, k=2')
                self.rotate_right()
                self.move_forward()
                self.rotate_left()
                self.move_forward()
                self.move_forward()
                self.stop()
            if k==3: #우회전-직진-[정지]
                self.get_logger().info('m1<m2 and n1<n2, k=3')
                self.rotate_right()
                self.move_forward()
                self.stop()
            if k==4: #좌회전-직진-좌회전-[정지]
                self.get_logger().info('m1<m2 and n1<n2, k=4')
                self.rotate_left()
                self.move_forward()
                self.rotate_left()
                self.stop()
        elif m1<m2 and n1>n2:
            if k==1: #우회전-[정지]
                self.get_logger().info('m1<m2 and n1>n2, k=1')
                self.rotate_right()
                self.stop()
            if k==2: #좌회전-직진-우회전-직진-[정지]
                self.get_logger().info('m1<m2 and n1>n2, k=2')
                self.rotate_left()
                self.move_forward()
                self.rotate_right()
                self.move_forward()
                self.stop()
            if k==3: #좌회전-직진-우회전-직진-직진-우회전-직진-[정지]
                self.get_logger().info('m1<m2 and n1>n2, k=3')
                self.rotate_left()
                self.move_forward()
                self.rotate_right()
                self.move_forward()
                self.move_forward()
                self.rotate_right()
                self.move_forward()
                self.stop()
            if k==4: #우회전-직진-우회전-후진진-[정지]
                self.get_logger().info('m1<m2 and n1>n2, k=4')
                self.rotate_right()
                self.move_forward()
                self.rotate_right()
                self.move_backward()
                self.stop()     
        elif m1>m2 and n1<n2:
            if k==1: #우회전-직진-좌회전-[정지]
                self.get_logger().info('m1>m2 and n1<n2, k=1')
                self.rotate_right()
                self.move_forward()
                self.rotate_left()
                self.stop()
            if k==2: #우회전-후진-[정지]
                self.get_logger().info('m1>m2 and n1<n2, k=2')
                self.rotate_right()
                self.move_backward()
                self.stop()
            if k==3: #좌회전-직진-좌회전-후진-후진-[정지]
                self.get_logger().info('m1>m2 and n1<n2, k=3')
                self.rotate_left()
                self.move_forward()
                self.rotate_left()
                self.move_backward()
                self.move_backward()
                self.stop()
            if k==4: #우회전-직진-좌회전-직진-직진-좌회전-[정지]
                self.get_logger().info('m1>m2 and n1<n2, k=4')
                self.rotate_right()
                self.move_forward()
                self.rotate_left()
                self.move_forward()
                self.move_forward()
                self.rotate_left()
                self.stop()
        else:
            self.get_logger().info('입력값 오류입니다.')
        self.get_logger().info('escape 함수 완료')

    def __del__(self):
        del self.yolo_detector