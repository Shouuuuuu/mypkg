import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class PomodoroTimer(Node):

    def __init__(self):
        super().__init__('pomodoro_timer')
        self.work_duration = 25 * 60
        self.current_time = self.work_duration
        self.is_running = False
        self.publisher_ = self.create_publisher(String, 'mypkg/countdown', 10)
        self.srv = self.create_service(
            SetBool, 'mypkg/control', self.control_callback)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('mypkg Pomodoro Timer Ready.')

    def control_callback(self, request, response):
        if request.data:
            if self.current_time <= 0:
                self.current_time = self.work_duration
            self.is_running = True
            response.success = True
            response.message = "Timer Started"
        else:
            self.is_running = False
            response.success = True
            response.message = "Timer Paused"
        return response

    def timer_callback(self):
        if self.is_running and self.current_time > 0:
            self.current_time -= 1
            mins, secs = divmod(self.current_time, 60)
            msg = String()
            msg.data = f"Remaining: {mins:02d}:{secs:02d}"
            self.publisher_.publish(msg)
            if self.current_time == 0:
                self.is_running = False
                self.get_logger().info("Time up!")


def main(args=None):
    rclpy.init(args=args)
    node = PomodoroTimer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
