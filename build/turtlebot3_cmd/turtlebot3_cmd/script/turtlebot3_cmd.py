#!/usr/bin/env python3
import rclpy
from .movement import TurtleBotController


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()

    try:  
        
        # 시연 코드
        controller.move_0X3() #첫번째 목표로 이동
        controller.move_detection(1,4,1) #사진 위치로 이동
        controller.stop()
        controller.detect(5.0, 'bus') #사진 찍기

        controller.escape(1,4,1,3,6) # 빠져나오기 + 두번째 목표로 이동
        controller.move_detection(3,6,1) #사진 위치로 이동
        controller.stop()
        controller.detect(5.0, 'airplane') #사진 찍기
       
        controller.escape(3,6,1,5,2) # 빠져나오기
        controller.rotate_left() # 좌표계 재설정
        controller.move_2X1()  # 세번째목표로 이동
        controller.move_detection(5,5,2)
        controller.stop()
        controller.detect(0.5, 'bear') # 사진찍기
        controller.escape(5,5,2,8,1) # 빠져나오기
        controller.rotate_left() # 좌표계 재설정
        controller.move_4X2() # 시작점 복귀

        """           
        # 다른 예시
        controller.move_3X0()
        controller.move_detection(4,1,3)
        controller.detect(0.5, 'bear')

        controller.escape(4,1,3,4,3)
        controller.rotate_left()
        controller.move_detection(4,3,1)
        controller.detect(5.0, 'bus')

        controller.escape(4,3,1,7,6)
        controller.move_1X1()
        controller.move_detection(7,6,3)
        controller.detect(5.0, 'airplane')

        controller.rotate_left()
        controller.move_forward()
        controller.move_3X3()
        controller.stop()
        controller.move_3X3()
        controller.stop()
        controller.move_1X2()
        """

    except KeyboardInterrupt:
        controller.get_logger().info('사용자에 의한 중단, 정지합니다...')
        controller.stop()
    except Exception as e:
        controller.get_logger().error(f'오류 발생: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()