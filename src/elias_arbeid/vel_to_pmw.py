import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from wg_interface.msg import ControlEvent
import RPi.GPIO as GPIO
from wg_utilities.data_utilities import headers  # type: ignore[import-not-found]

class VelToPwmNode(Node):
    def __init__(self):
        super().__init__('vel_to_pwm_node')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom/raw')
        self.declare_parameter('control_event_topic', '/control_event')
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('wheel_diameter', 0.055)   # meters (5.5 cm)
        self.declare_parameter('ticks_per_rev', 2069)
        self.declare_parameter('wheel_base', 0.3)         # meters
        self.declare_parameter('pwm_freq', 100)           # Hz
        self.declare_parameter('kp', 50.0)
        self.declare_parameter('kd', 5.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.control_event_topic = self.get_parameter('control_event_topic').get_parameter_value().string_value
        self.max_pwm = int(self.get_parameter('max_pwm').get_parameter_value().integer_value)
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').get_parameter_value().double_value)
        self.ticks_per_rev = int(self.get_parameter('ticks_per_rev').get_parameter_value().integer_value)
        self.wheel_base = float(self.get_parameter('wheel_base').get_parameter_value().double_value)
        self.pwm_freq = int(self.get_parameter('pwm_freq').get_parameter_value().integer_value)
        self.kp = float(self.get_parameter('kp').get_parameter_value().double_value)
        self.kd = float(self.get_parameter('kd').get_parameter_value().double_value)

        # Precomputed scale: ticks per meter
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.ticks_per_meter = self.ticks_per_rev / self.wheel_circumference  # ticks / m

        # GPIO setup (do once)
        GPIO.setmode(GPIO.BCM)
        self.gpio_left_pin = 12
        self.gpio_right_pin = 13
        GPIO.setup(self.gpio_left_pin, GPIO.OUT)
        GPIO.setup(self.gpio_right_pin, GPIO.OUT)
        self._pwm_left = GPIO.PWM(self.gpio_left_pin, self.pwm_freq)
        self._pwm_right = GPIO.PWM(self.gpio_right_pin, self.pwm_freq)
        self._pwm_left.start(0)
        self._pwm_right.start(0)

        # QoS and subscriptions
        qos = QoSProfile(depth=10)
        self.cmd_vel_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos)
        self.control_event_pub = self.create_publisher(ControlEvent, self.control_event_topic, qos)

        # PD control state
        self._prev_error_left = 0.0
        self._prev_error_right = 0.0
        self._last_control_time = self.get_clock().now()

        # Feedback from odometry
        self._actual_linear_vel = 0.0
        self._actual_angular_vel = 0.0

        # Command setpoints
        self._cmd_linear_vel = 0.0
        self._cmd_angular_vel = 0.0

    def odom_callback(self, msg: Odometry):
        self._actual_linear_vel = msg.twist.twist.linear.x
        self._actual_angular_vel = msg.twist.twist.angular.z

    def pd_control(self, desired_vel, actual_vel, prev_error, current_time):
        error = desired_vel - actual_vel
        dt = (current_time - self._last_control_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 0.01

        d_error = (error - prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.kd * d_error

        pwm = int(max(min(output, self.max_pwm), -self.max_pwm))
        return pwm, error

    def estimate_wheel_velocities(self):
        v_linear = self._actual_linear_vel
        v_angular = self._actual_angular_vel
        wheel_base = self.wheel_base

        v_left_actual = v_linear - (v_angular * wheel_base / 2.0)
        v_right_actual = v_linear + (v_angular * wheel_base / 2.0)
        return v_left_actual, v_right_actual

    def compute_wheel_velocities(self):
        v_left = self._cmd_linear_vel - (self._cmd_angular_vel * self.wheel_base / 2.0)
        v_right = self._cmd_linear_vel + (self._cmd_angular_vel * self.wheel_base / 2.0)
        return v_left, v_right

    def apply_pwm(self, pwm_left, pwm_right):
        # PWM library expects duty cycle 0..100; convert from -max..max to direction + duty
        def to_duty_and_dir(pwm):
            sign = 1 if pwm >= 0 else -1
            duty = min(abs(pwm) / self.max_pwm * 100.0, 100.0)
            return duty, sign

        duty_l, dir_l = to_duty_and_dir(pwm_left)
        duty_r, dir_r = to_duty_and_dir(pwm_right)

        # Here you should set motor direction pins accordingly (not shown).
        # For simplicity, assume motor driver handles sign via separate pins or H-bridge wiring.
        self._pwm_left.ChangeDutyCycle(duty_l)
        self._pwm_right.ChangeDutyCycle(duty_r)

    def cmd_vel_callback(self, msg: Twist):
        self._cmd_linear_vel = float(msg.linear.x)
        self._cmd_angular_vel = float(msg.angular.z)

        v_left_cmd, v_right_cmd = self.compute_wheel_velocities()
        v_left_actual, v_right_actual = self.estimate_wheel_velocities()

        current_time = self.get_clock().now()

        pwm_left, error_left = self.pd_control(v_left_cmd, v_left_actual, self._prev_error_left, current_time)
        pwm_right, error_right = self.pd_control(v_right_cmd, v_right_actual, self._prev_error_right, current_time)

        self._prev_error_left = error_left
        self._prev_error_right = error_right
        self._last_control_time = current_time

        self.apply_pwm(pwm_left, pwm_right)

        control_event_msg = ControlEvent()
        control_event_msg.header.stamp = self.get_clock().now().to_msg()
        control_event_msg.header.frame_id = 'base_link'
        control_event_msg.pwm_left = pwm_left
        control_event_msg.pwm_right = pwm_right
        self.control_event_pub.publish(control_event_msg)
        self.get_logger().info(f'PD Control -> pwm L:{pwm_left} R:{pwm_right} (cmd_v={self._cmd_linear_vel:.3f} m/s cmd_w={self._cmd_angular_vel:.3f} rad/s)')

    def destroy_node(self):
        # stop PWM and cleanup GPIO on shutdown
        try:
            self._pwm_left.stop()
            self._pwm_right.stop()
            GPIO.cleanup()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VelToPwmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
