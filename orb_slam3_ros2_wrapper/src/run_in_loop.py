import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SquareLoopPublisher(Node):
    def __init__(self):
        super().__init__('square_loop_publisher')
        self.publisher_ = self.create_publisher(Twist, '/robot_0/cmd_vel_nav', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.side_length = 10.0  # 10 meters per side
        self.linear_speed = 0.5  # Adjust as needed
        self.angular_speed = math.pi / 2  # 90 degrees per second
        self.current_side = 0
        self.side_duration = self.side_length / self.linear_speed
        self.turn_duration = 1.0  # 1 second to make a 90-degree turn
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.state = 'moving'

    def timer_callback(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time

        twist = Twist()

        if self.state == 'moving':
            if elapsed_time < self.side_duration:
                twist.linear.x = self.linear_speed
            else:
                self.state = 'turning'
                self.start_time = current_time
        elif self.state == 'turning':
            if elapsed_time < self.turn_duration:
                twist.angular.z = self.angular_speed
            else:
                self.current_side += 1
                if self.current_side < 4:
                    self.state = 'moving'
                    self.start_time = current_time
                else:
                    self.get_logger().info('Completed square loop')
                    self.destroy_node()
                    return

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    square_loop_publisher = SquareLoopPublisher()
    rclpy.spin(square_loop_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()