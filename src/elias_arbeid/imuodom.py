#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import board  # type: ignore[import-not-found]
import busio  # type: ignore[import-not-found]

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

from adafruit_bno08x import (  # type: ignore[import-not-found]
	BNO_REPORT_ACCELEROMETER,
	BNO_REPORT_GYROSCOPE,
	BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore[import-not-found]


class ImuOdom(Node):
	def __init__(self):
		super().__init__('imu_odom_node')

		self.declare_parameter('topic_name', '/imu/data')
		self.declare_parameter('frame_id', 'imu_link')
		self.declare_parameter('publish_hz', 100.0)
		self.declare_parameter('i2c_address', 0x4A)

		self.topic_name = str(self.get_parameter('topic_name').value or '/imu/data')
		self.frame_id = str(self.get_parameter('frame_id').value or 'imu_link')
		self.publish_hz = float(self.get_parameter('publish_hz').value or 100.0)
		self.i2c_address = int(self.get_parameter('i2c_address').value or 0x4A)

		self.publisher = self.create_publisher(Imu, self.topic_name, 10)

		self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
		self.imu = BNO08X_I2C(self.i2c, address=self.i2c_address)

		report_interval_us = int(1_000_000 / self.publish_hz)
		self.imu.enable_feature(BNO_REPORT_ROTATION_VECTOR, report_interval_us)
		self.imu.enable_feature(BNO_REPORT_ACCELEROMETER, report_interval_us)
		self.imu.enable_feature(BNO_REPORT_GYROSCOPE, report_interval_us)

		self.timer = self.create_timer(1.0 / self.publish_hz, self.publish_imu)
		self._warned_not_ready = False
		self.get_logger().info(f'Publishing IMU data on {self.topic_name} from I2C address 0x{self.i2c_address:02x}')

	def publish_imu(self):
		quaternion = self.imu.quaternion
		acceleration = self.imu.linear_acceleration
		gyro = self.imu.gyro

		if quaternion is None or acceleration is None or gyro is None:
			if not self._warned_not_ready:
				self.get_logger().warn('IMU sample not ready yet')
				self._warned_not_ready = True
			return
		self._warned_not_ready = False

		quat_i, quat_j, quat_k, quat_real = quaternion
		accel_x, accel_y, accel_z = acceleration
		gyro_x, gyro_y, gyro_z = gyro

		msg = Imu()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = self.frame_id

		msg.orientation = Quaternion(
			x=float(quat_i),
			y=float(quat_j),
			z=float(quat_k),
			w=float(quat_real),
		)
		msg.angular_velocity.x = float(gyro_x)
		msg.angular_velocity.y = float(gyro_y)
		msg.angular_velocity.z = float(gyro_z)
		msg.linear_acceleration.x = float(accel_x)
		msg.linear_acceleration.y = float(accel_y)
		msg.linear_acceleration.z = float(accel_z)

		msg.orientation_covariance = [
			0.02, 0.0, 0.0,
			0.0, 0.02, 0.0,
			0.0, 0.0, 0.02,
		]
		msg.angular_velocity_covariance = [
			0.01, 0.0, 0.0,
			0.0, 0.01, 0.0,
			0.0, 0.0, 0.01,
		]
		msg.linear_acceleration_covariance = [
			0.1, 0.0, 0.0,
			0.0, 0.1, 0.0,
			0.0, 0.0, 0.1,
		]

		self.publisher.publish(msg)


def main(args=None):
	rclpy.init(args=args)
	node = ImuOdom()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
