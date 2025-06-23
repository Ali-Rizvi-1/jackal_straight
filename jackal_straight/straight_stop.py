#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math, time

class StraightStopNode(Node):
    def __init__(self):
        super().__init__('straight_stop')
        # parameters
        self.declare_parameter('distance_threshold', 0.9)
        self.declare_parameter('forward_speed',     0.1)
        self.declare_parameter('stop_duration',    60.0)
        self.dist_thresh = self.get_parameter('distance_threshold').value
        self.speed       = self.get_parameter('forward_speed').value
        self.stop_dur    = self.get_parameter('stop_duration').value

        # state
        self.state     = 'MOVING'
        self.start_x   = None
        self.start_y   = None
        self.distance  = 0.0
        self.stop_time = None

        # publishers / subscribers
        self.cmd_pub  = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'platform/odom', self.odom_cb, 10)

        # periodic timer to send velocity commands
        self.timer = self.create_timer(0.1, self.timer_cb)

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x, self.start_y = x, y
            return
        dx = x - self.start_x
        dy = y - self.start_y
        self.distance = math.hypot(dx, dy)

    def timer_cb(self):
        twist = Twist()
        now = time.time()
        if self.state == 'MOVING':
            if self.distance >= self.dist_thresh:
                self.get_logger().info(f'Reached {self.distance:.3f} m; stopping')
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                self.state = 'STOPPED'
                self.stop_time = now
            else:
                twist.linear.x = self.speed
                self.cmd_pub.publish(twist)
        else:  # STOPPED
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            if now - self.stop_time >= self.stop_dur:
                self.get_logger().info('Stop complete; resuming')
                self.state    = 'MOVING'
                self.start_x  = None
                self.distance = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = StraightStopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
