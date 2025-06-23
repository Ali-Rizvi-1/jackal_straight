#!/usr/bin/env python3
# heading_maintainer.py, at the very top:
import numpy as np
# restore the old np.float alias so transforms3d won’t crash
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import tf_transformations
import math
import time

class HeadingMaintainer(Node):
    def __init__(self):
        super().__init__('heading_maintainer')
        # PID gains (tune these)
        self.declare_parameter('P', 4.0)
        self.declare_parameter('I', 0.0)
        self.declare_parameter('D', 0.1)
        self.P = self.get_parameter('P').value
        self.I = self.get_parameter('I').value
        self.D = self.get_parameter('D').value

        self.yaw_target = None
        self.yaw_error_sum = 0.0
        self.last_error   = 0.0
        self.last_time    = None

        # subscribers & publisher
        self.create_subscription(Imu, 'sensors/imu_0/data', self.imu_cb, 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self.raw_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def imu_cb(self, msg: Imu):
        # extract yaw from quaternion
        q = msg.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])

        if self.yaw_target is None:
            # lock in our “straight” heading at startup
            self.yaw_target = yaw
            self.last_time  = time.time()
            return

        # store current yaw for error calc in raw_cb
        self.current_yaw = yaw

    def raw_cb(self, raw: Twist):
        # still no valid yaw to correct? just passthrough
        if self.yaw_target is None or self.current_yaw is None:
            # no heading reference yet → just forward
            out = Twist()
            out.linear = raw.linear
            self.cmd_pub.publish(out)
            return

        now = time.time()
        dt = now - self.last_time if self.last_time else 0.02
        error = self._angle_diff(self.yaw_target, self.current_yaw)

        # PID
        self.yaw_error_sum += error * dt
        de = (error - self.last_error) / dt if dt>0 else 0.0
        control = self.P*error + self.I*self.yaw_error_sum + self.D*de

        # build output twist
        out = Twist()
        out.linear.x  = raw.linear.x
        out.linear.y  = raw.linear.y
        out.linear.z  = raw.linear.z
        out.angular.z = control

        self.cmd_pub.publish(out)

        # update history
        self.last_error = error
        self.last_time  = now

    @staticmethod
    def _angle_diff(target, current):
        """Compute signed smallest angle difference."""
        a = target - current
        return math.atan2(math.sin(a), math.cos(a))

def main(args=None):
    rclpy.init(args=args)
    node = HeadingMaintainer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
