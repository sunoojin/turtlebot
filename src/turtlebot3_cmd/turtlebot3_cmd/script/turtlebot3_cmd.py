#!/usr/bin/env python3
import rclpy
from .movement import TurtleBotController


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()

    try:
        """
        controller.move_1X0()
        controller.detection(2,1,4)
        controller.escape(2,1,4,2,5)
        controller.move_2X0()
        controller.rotate_left()
        controller.detection(2,5,3)
        """
        # controller.escape(2,5,3,7,3)
        # controller.rotate_left()
        # controller.move_2X4()
        # controller.rotate_right()
        #controller.detection(7,3,3)
        controller.start_detection(5.0)
        #코드를 짜는 부분

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