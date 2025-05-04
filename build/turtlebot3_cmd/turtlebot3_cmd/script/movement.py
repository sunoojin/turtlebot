import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # 로봇 제어 관련 초기화
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.command = Twist()

        # YOLO 물체 감지 객체 초기화
        self.subscription = self.create_subscription(
            Image,
            'webcam/image',
            self.image_callback,
            1
        )
        self.model = YOLO('/home/root2/turtlebot3_ws/yolov8n.pt')

        # OpenCV 브리지 객체 생성
        self.bridge = CvBridge()

        # 저장 경로 생성 (결과 디렉토리 지정)
        self.save_dir = "/home/root2/turtlebot3_ws/result"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.detect_mode = False
        self.detect_start_time = 0
        self.detect_duration = 0
        self.detected_objects = set()
        self.detection_complete = False
        self.target_class = None


    def image_callback(self, msg):
        count =0
        # ROS Image 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        print(time.time() - self.detect_start_time)
        print(self.detect_mode)
        if self.detect_mode:
            # 감지 시간이 초과되면 종료
            if time.time() - self.detect_start_time > self.detect_duration:
                self.detect_mode = False
                self.detection_complete = True
                self.get_logger().info('물체 감지가 완료되었습니다.')
                return
            
            # YOLO 감지 수행
            results = self.model(frame)
            print(results)
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]

                    if cls_name != self.target_class:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    obj_img = frame[y1:y2, x1:x2]
                    if obj_img.size > 0:
                        filename = os.path.join(self.save_dir, f"{cls_name}_{time.time():.2f}.jpg")
                        cv2.imwrite(filename, obj_img)
                        self.detected_objects.add(cls_name)
                        self.get_logger().info(f'물체 저장됨: {filename}')
                        self.detection_complete=True
            #             break                      
            count+=1            
        cv2.imshow('image',frame)
        cv2.waitKey(1)

    # 탐지함수
    def detect(self, duration, target_class='person'):
        self.detect_mode = True
        self.detect_start_time = time.time()
        self.detect_duration = duration
        self.detected_objects = set()
        self.detection_complete = False
        self.target_class = target_class
        self.get_logger().info(f'{duration}초 동안 {target_class} 탐지를 시작합니다.')
        
        while not self.detection_complete and rclpy.ok():
            rclpy.spin_once(self,timeout_sec=0.1)

    def timer_callback(self):
        self.publisher.publish(self.command)

    # 기본 이동함수
    def move_forward(self, linear_speed=0.2, duration=2.5):
        if linear_speed < 0:
            raise ValueError('회전 속도는 양수이어야 합니다.')
        self.command.linear.x = linear_speed
        self.command.angular.z = 0.0
        self._publish_for_duration(duration)
    
    def move_forward_diagonal(self, linear_speed=0.2, duration=3.4):
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
    
    def rotate_left(self, angular_speed=0.5, duration=3.2):
        if angular_speed < 0:
            raise ValueError('회전 속도는 양수이어야 합니다.')
        self.command.linear.x = 0.0
        self.command.angular.z = angular_speed
        self._publish_for_duration(duration)

    def rotate_right(self, angular_speed=0.5, duration=3.4):
        if angular_speed < 0:
            raise ValueError('회전 속도는 양수이어야 합니다.')
        self.command.linear.x = 0.0
        self.command.angular.z = -angular_speed
        self._publish_for_duration(duration)

    def rotate_left_diagonal(self, angular_speed=0.5, duration=1.8):
        if angular_speed < 0:
            raise ValueError('회전 속도는 양수이어야 합니다.')
        self.command.linear.x = 0.0
        self.command.angular.z = angular_speed
        self._publish_for_duration(duration)
    
    def rotate_right_diagonal(self, angular_speed=0.5, duration=2.0):
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

    #사진 위치로 이동 함수
    def move_detection(self, m, n, k):
        """매개 변수: 상자 좌표 = (m, n), 사진 위치 = k"""
        self.get_logger().info('detection 함수 시작')
        if m == 0:
            if k == 1:  # 우회전-정지
                self.get_logger().info('m=0, k=1')
                self.rotate_right()
                self.stop()

            if k == 3:  # 직진-우회전-직진-직진-우회전-직진-우회전-[정지]
                self.get_logger().info('m=0, k=3')
                self.move_forward()
                self.rotate_right()
                self.move_forward()
                self.move_forward()
                self.rotate_right()
                self.move_forward()
                self.rotate_right()
                self.stop()

            if k == 4:  # 직진-우회전-직진-우회전-[정지]
                self.get_logger().info('m=0, k=4')
                self.move_forward()
                self.rotate_right()
                self.move_forward()
                self.rotate_right()
                self.stop()

        elif n == 0:
            if k == 2:  # [정지]
                self.get_logger().info('n=0, k=2')
                self.stop()

            if k == 3:  # 우회전-직진-좌회전-직진-좌회전-[정지]
                self.get_logger().info('n=0, k=3')
                self.rotate_right()
                self.move_forward()
                self.rotate_left()
                self.move_forward()
                self.rotate_left()
                self.stop()

            if k == 4:  # 우회전-직진-좌회전-직진-직진-좌회전-직진-좌회전-[정지]
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

        else:
            if k == 1:  # 직진-우회전-[정지]
                self.get_logger().info('else, k=1')
                self.move_forward()
                self.rotate_right()
                self.stop()

            if k == 2:  # 우회전-직진-좌회전-[정지]
                self.get_logger().info('else, k=2')
                self.rotate_right()
                self.move_forward()
                self.rotate_left()
                self.stop()

            if k == 3:  # 우회전-직진-직진-좌회전-직진-좌회전-[정지]
                self.get_logger().info('else, k=3')
                self.rotate_right()
                self.move_forward()
                self.move_forward()
                self.rotate_left()
                self.move_forward()
                self.rotate_left()
                self.stop()

            if k == 4:  # 직진-직진-우회전-직진-우회전-[정지]
                self.get_logger().info('else, k=4')
                self.move_forward()
                self.move_forward()
                self.rotate_right()
                self.move_forward()
                self.rotate_right()
                self.stop()
        self.get_logger().info('detection 함수 완료')

    # 다음 목표로 이동하기 위한 가장 안전한 위치로 이동하는 함수
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

            elif m1 > m2:  #예외사항
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
            if k==1: #좌회전-[정지]
                self.get_logger().info('m1<m2 and n1>n2, k=1')
                self.rotate_left()
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