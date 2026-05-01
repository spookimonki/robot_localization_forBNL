#!/usr/bin/env python3
"""LD06 LiDAR ROS 2 node over the Raspberry Pi UART on pins 8/10.

Reads LD06 packets from the Pi serial alias and publishes LaserScan messages.
"""

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import RPi.GPIO as GPIO  # type: ignore[import-not-found]
import serial  # type: ignore[import-not-found]


LD06_PACKET_LEN = 47


def crc8_ld06(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x1D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


@dataclass
class LidarPacket:
    speed_deg_s: float
    start_angle_deg: float
    end_angle_deg: float
    points: list[dict]
    timestamp_ms: int
    checksum_match: bool
    raw: bytes


class LD06Parser:
    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[LidarPacket]:
        out: list[LidarPacket] = []
        self._buf.extend(data)
        while True:
            if len(self._buf) < LD06_PACKET_LEN:
                break
            try:
                start_idx = self._buf.index(0x54)
            except ValueError:
                self._buf.clear()
                break

            if start_idx > 0:
                del self._buf[:start_idx]

            if len(self._buf) < LD06_PACKET_LEN:
                break

            if self._buf[1] != 0x2C:
                del self._buf[0]
                continue

            pkt = bytes(self._buf[:LD06_PACKET_LEN])
            del self._buf[:LD06_PACKET_LEN]

            checksum_match = crc8_ld06(pkt[:-1]) == pkt[-1]
            speed = int.from_bytes(pkt[2:4], "little")
            start_ang = int.from_bytes(pkt[4:6], "little") / 100.0
            points: list[dict] = []
            off = 6
            for _ in range(12):
                dist = int.from_bytes(pkt[off : off + 2], "little")
                inten = pkt[off + 2]
                points.append({"range_mm": dist, "intensity": int(inten)})
                off += 3
            end_ang = int.from_bytes(pkt[42:44], "little") / 100.0
            ts_ms = int.from_bytes(pkt[44:46], "little")

            out.append(
                LidarPacket(
                    speed_deg_s=float(speed),
                    start_angle_deg=float(start_ang),
                    end_angle_deg=float(end_ang),
                    points=points,
                    timestamp_ms=int(ts_ms),
                    checksum_match=checksum_match,
                    raw=pkt,
                )
            )
        return out


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        GPIO.setwarnings(False)

        self.declare_parameter('topic_name', '/scan')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baudrate', 230400)

        self.topic_name = str(self.get_parameter('topic_name').value or '/scan')
        self.frame_id = str(self.get_parameter('frame_id').value or 'lidar_link')
        self.publish_hz = float(self.get_parameter('publish_hz').value or 30.0)
        self.port = str(self.get_parameter('port').value or '/dev/serial0')
        self.baudrate = int(self.get_parameter('baudrate').value or 230400)

        self.publisher = self.create_publisher(LaserScan, self.topic_name, 10)

        self.parser = LD06Parser()
        self.ser: serial.Serial | None = None
        self._connection_failed_warning = False

        self.timer = self.create_timer(1.0 / self.publish_hz, self.publish_scan)

        self.get_logger().info(
            f'LiDAR node initialized: {self.topic_name} @ {self.publish_hz:.1f}Hz, '
            f'port={self.port} baudrate={self.baudrate}'
        )

    def _connect(self) -> bool:
        try:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f'Connected to LiDAR on {self.port} @ {self.baudrate} baud')
            self._connection_failed_warning = False
            return True
        except Exception as exc:
            if not self._connection_failed_warning:
                self.get_logger().error(f'Failed to connect to LiDAR: {exc}')
                self._connection_failed_warning = True
            return False

    def publish_scan(self) -> None:
        if self.ser is None:
            if not self._connect():
                return

        ser = self.ser
        if ser is None:
            return

        try:
            chunk = ser.read(1024)
            if not chunk:
                return

            packets = self.parser.feed(chunk)
            if not packets:
                return

            for pkt in packets:
                scan = LaserScan()
                scan.header.stamp = self.get_clock().now().to_msg()
                scan.header.frame_id = self.frame_id

                start_rad = math.radians(pkt.start_angle_deg)
                end_rad = math.radians(pkt.end_angle_deg)
                if end_rad < start_rad:
                    end_rad += 2.0 * math.pi

                scan.angle_min = start_rad
                scan.angle_max = end_rad
                scan.angle_increment = (end_rad - start_rad) / (len(pkt.points) - 1) if len(pkt.points) > 1 else 0.0
                scan.range_min = 0.05
                scan.range_max = 12.0

                scan.ranges = [float(p["range_mm"]) / 1000.0 for p in pkt.points]
                scan.intensities = [float(p["intensity"]) for p in pkt.points]

                scan.time_increment = 0.0
                scan.scan_time = 1.0 / self.publish_hz if self.publish_hz > 0.0 else 0.0

                self.publisher.publish(scan)

        except Exception as exc:
            self.get_logger().error(f'Error reading LiDAR data: {exc}')
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    finally:
        GPIO.cleanup()
        if node.ser is not None:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
