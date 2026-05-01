#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import RPi.GPIO as GPIO  # type: ignore[import-not-found]
import time


class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom_node')

        self.declare_parameter('odom_topic', '/odom/raw')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.odom_topic = str(self.get_parameter('odom_topic').value or '/odom/raw')
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value or 'odom')
        self.base_frame_id = str(self.get_parameter('base_frame_id').value or 'base_link')

        # 1. Robot parameters
        self.wheel_radius = 0.05
        self.wheel_base = 0.3
        self.encoder_resolution_m1 = 2048
        self.encoder_resolution_m2 = 2048

        # 2. Odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # encoder counts
        self.encoder1_count = 0
        self.encoder2_count = 0
        self.prev_encoder1_count = 0
        self.prev_encoder2_count = 0

        self.last_time = time.time()

        # 3. ROS publisher and timer
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_odom)

        # 4. GPIO setup
        GPIO.setmode(GPIO.BCM)
        self.encoder1_pin_a = 4
        self.encoder1_pin_b = 17
        self.encoder2_pin_a = 5
        self.encoder2_pin_b = 6

        GPIO.setup(self.encoder1_pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encoder1_pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encoder2_pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encoder2_pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # previous A states for quadrature decoding (after pins are configured)
        self.encoder1_a_last = GPIO.input(self.encoder1_pin_a)
        self.encoder2_a_last = GPIO.input(self.encoder2_pin_a)

        GPIO.add_event_detect(self.encoder1_pin_a, GPIO.BOTH, callback=self.encoder1_callback)
        GPIO.add_event_detect(self.encoder1_pin_b, GPIO.BOTH, callback=self.encoder1_callback)
        GPIO.add_event_detect(self.encoder2_pin_a, GPIO.BOTH, callback=self.encoder2_callback)
        GPIO.add_event_detect(self.encoder2_pin_b, GPIO.BOTH, callback=self.encoder2_callback)

    # 5. Encoder callbacks
    def encoder1_callback(self, channel):
        a_state = GPIO.input(self.encoder1_pin_a)

        if a_state != self.encoder1_a_last:
            b_state = GPIO.input(self.encoder1_pin_b)

            if b_state != a_state:
                self.encoder1_count += 1
            else:
                self.encoder1_count -= 1

            self.encoder1_a_last = a_state


    def encoder2_callback(self, channel):
        a_state = GPIO.input(self.encoder2_pin_a)

        if a_state != self.encoder2_a_last:
            b_state = GPIO.input(self.encoder2_pin_b)

            if b_state != a_state:
                self.encoder2_count += 1
            else:
                self.encoder2_count -= 1

            self.encoder2_a_last = a_state

    # 6. Conversion helpers
    def m1_ticks_to_distance(self, ticks):
        return (2 * math.pi * self.wheel_radius) * (ticks / self.encoder_resolution_m1)

    def m2_ticks_to_distance(self, ticks):
        return (2 * math.pi * self.wheel_radius) * (ticks / self.encoder_resolution_m2)

    def delta_theta(self, distance_left, distance_right):
        return (distance_right - distance_left) / self.wheel_base

    # 7. Odometry integration
    def integrate_odometry(self, distance_left, distance_right):
        d = (distance_left + distance_right) / 2.0
        dtheta = self.delta_theta(distance_left, distance_right)

        if abs(dtheta) < 1e-9:
            self.x += d * math.cos(self.yaw)
            self.y += d * math.sin(self.yaw)
        else:
            radius = d / dtheta
            self.x += radius * (math.sin(self.yaw + dtheta) - math.sin(self.yaw))
            self.y -= radius * (math.cos(self.yaw + dtheta) - math.cos(self.yaw))

        self.yaw += dtheta
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    # 8. Publish odometry
    def publish_odom(self):
        now = time.time()
        delta_time = now - self.last_time
        if delta_time <= 0.0:
            return

        left_ticks = self.encoder1_count - self.prev_encoder1_count
        right_ticks = self.encoder2_count - self.prev_encoder2_count

        distance_left = self.m1_ticks_to_distance(left_ticks)
        distance_right = self.m2_ticks_to_distance(right_ticks)

        self.integrate_odometry(distance_left, distance_right)

        linear_velocity = ((distance_left + distance_right) / 2.0) / delta_time
        angular_velocity = self.delta_theta(distance_left, distance_right) / delta_time

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.twist.twist.linear.x = linear_velocity
        msg.twist.twist.angular.z = angular_velocity

        msg.pose.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.04,
        ]
        msg.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
        ]

        self.odom_pub.publish(msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.odom_frame_id
        tf_msg.child_frame_id = self.base_frame_id
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        self.prev_encoder1_count = self.encoder1_count
        self.prev_encoder2_count = self.encoder2_count
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdom()
    try:
        rclpy.spin(node)
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

