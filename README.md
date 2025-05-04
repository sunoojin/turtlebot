# turtlebot
CNN 기반 이미지 인식 모델인 YOLO를 사용하여 터틀봇이 움직이며 사진을 식별하도록 설계하는 프로젝트를 진행했습니다.

# 역할분담

| 팀장 | 라치현 | 세종대학교 | 알고리즘 설계, 터틀봇 진행로 시뮬레이션 |
| --- | --- | --- | --- |
| 팀원 | 반제호 | 국민대학교 | 알고리즘 구현 |
| 팀원 | 오정인 | 동국대학교 | 개발환경 설정, Python 코딩, 터틀봇 진행로 시뮬레이션 |
| 팀원 | 조선주 | 대전대학교 | 발표자료 제작 |

# 개발 환경

ROS2, Python

# 함수 설명

### turtlebot3_cmd.py

`main 함수`

다른 예시는 발표에서 언급했던 조교님이 랜덤으로 설정해주신 경우의 코드를 의미합니다.

해당 영상: https://youtube.com/shorts/vSTquBoTK-Y?feature=shared

### movement.py

- `detect`: 주어진 subject를 5초간 인식하고 사진으로 저장
- `move_forward`: 앞으로 한칸 이동
- `move_forward_diagonal`: 대각선 한칸 이동
- `rotate_left`: 왼쪽 90도 회전
- `rotate_right`: 오른쪽 90도 회전
- `rotate_left_diagonal`: 왼쪽  45도 회전
- `rotate_right_diagonal`: 오른쪽 45도 회전
- `stop`: 정지
- `move_1X1 ~ move_6X7` : 현재위치를 0X0으로 생각하고 해당 위치로 이동하는 함수
- `move_detection`: 사진을 찍을 수 있는 위치로 이동하는 함수
- `escape`: 다음 목표로 이동하기 위한 가장 안전한 위치로 이동하는 함수
