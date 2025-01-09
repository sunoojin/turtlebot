#!/usr/bin/env python3

from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO
import os
import time

class YoloDetector:
    def __init__(self):
        # YOLO 모델 초기화
        self.model = YOLO('yolov8n.pt')

        # 저장 경로 생성 (결과 디렉토리 지정)
        self.save_dir = os.path.join(os.path.dirname(__file__), '..', 'result')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # 감지 관련 변수
        self.detect_mode = False
        self.detect_start_time = 0
        self.detect_duration = 0
        self.detected_objects = set()
        self.detection_complete = False
        self.target_class = None

    def process_image(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if self.detect_mode:
            # 감지 시간이 초과되면 종료
            if time.time() - self.detect_start_time > self.detect_duration:
                self.detect_mode = False
                self.detection_complete = True
                print('물체 감지가 완료되었습니다.')
                return image

            # YOLO 감지 수행
            results = self.model(image)
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]

                    if cls_name != self.target_class:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    obj_img = image[y1:y2, x1:x2]
                    if obj_img.size > 0:
                        filename = os.path.join(self.save_dir, f"{cls_name}.jpg")
                        #cv.imshow(obj_img)
                        cv2.imwrite(filename, obj_img)
                        self.detected_objects.add(cls_name)
                        print(f'물체 저장됨: {filename}')

        return image

    def detect(self, duration, target_class='bottle'):
        self.detect_mode = True
        self.detect_start_time = time.time()
        self.detect_duration = duration
        self.detected_objects = set()
        self.detection_complete = False
        self.target_class = target_class
        print(f'{duration}초 동안 {target_class} 탐지를 시작합니다.')

        # ROS 메시지를 기다리면서 대기
        while True:
            time_elapsed = time.time() - self.detect_start_time

            # 감지 시간이 초과되면 detect_mode 강제 종료
            if time_elapsed > duration:
                self.detect_mode = False
                self.detection_complete = True
                print('감지 시간이 초과되었습니다. 감지 모드를 종료합니다.')
                break

            # 메시지가 도착할 가능성을 기다림
            time.sleep(0.1)

        print('감지 작업이 완료되었습니다.')


    def __del__(self):
        pass  # OpenCV 창 관련 코드 제거
