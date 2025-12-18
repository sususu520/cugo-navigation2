#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareMover(Node):
   def __init__(self):
       super().__init__('square_mover')
       self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
       self.timer_ = self.create_timer(1.0, self.move_square)
       self.step = 0
       self.max_steps = 8

   def move_square(self):
       if self.step >= self.max_steps:
           self.get_logger().info('正方形走行終了')
           self.destroy_timer(self.timer_)
           self.publish_stop()
           return

       twist = Twist()

       if self.step % 2 == 0:
           self.get_logger().info(f'直進中 (step {self.step})')
           twist.linear.x = 1.0  # m/s
           twist.angular.z = 0.0
           self.continuous_publish(twist, duration_sec=2.0)
       else:
           self.get_logger().info(f'右旋中 (step {self.step})')
           twist.linear.x = 0.0
           twist.angular.z = -1.57  # rad/s（90度旋转约需1秒）
           self.continuous_publish(twist, duration_sec=1.0)

       self.publish_stop()
       self.step += 1

   def continuous_publish(self, twist, duration_sec):
       start_time = self.get_clock().now().nanoseconds / 1e9
       end_time = start_time + duration_sec
       rate = 10  # Hz
       sleep_time = 1.0 / rate

       while (self.get_clock().now().nanoseconds / 1e9) < end_time:
           self.publisher_.publish(twist)
           time.sleep(sleep_time)

   def publish_stop(self):
       twist = Twist()
       twist.linear.x = 0.0
       twist.angular.z = 0.0
       self.publisher_.publish(twist)
       time.sleep(0.2)

def main(args=None):
   rclpy.init(args=args)
   node = SquareMover()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
