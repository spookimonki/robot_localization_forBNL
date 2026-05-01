# Elias Arbeid - Wheel Odometry, IMU, and LiDAR

This folder contains Python nodes for reading encoder, IMU, and LiDAR data from Raspberry Pi hardware and publishing ROS 2 topics.

## Structure

- `wheelodom.py` - Wheel encoder odometry node (quadrature decoder + pose integration)
- `imuodom.py` - IMU data reader over I2C
- `lidar_relay.py` - LD06 LiDAR node over the Raspberry Pi UART
- `measurements.yaml` - Robot physical parameters and hardware notes
- `README.md` - This file

## Quick Start

### Build

```bash
cd ~/Desktop/BNL
colcon build --packages-select robot_localization
```

### Run Full Localization Stack

```bash
source install/setup.bash
ros2 launch robot_localization full_localization.launch.py
```

This launches:
- Wheel odometry node (publishes `/odom/raw`)
- IMU node (publishes `/imu/data`)
- LiDAR node (publishes `/scan`)
- UKF filter (subscribes to `/odom/raw` + `/imu/data`, publishes `/odometry/filtered`)

### Check Topics

```bash
# Raw wheel odometry
ros2 topic echo /odom/raw

# Filtered odometry (UKF output)
ros2 topic echo /odometry/filtered

# IMU data
ros2 topic echo /imu/data

# LiDAR scan
ros2 topic echo /scan
```

## Robot Measurements

Edit `measurements.yaml` with your actual robot parameters once you hook up the vehicle:

```yaml
wheel_odom:
  wheel_radius: 0.05        # meters (update after measurement)
  wheel_base: 0.3           # meters (distance between wheels)
  encoder_resolution_m1: 360  # ticks per revolution
  encoder_resolution_m2: 360  # ticks per revolution

gpio_pins:
  encoder1_pin_a: 4         # BCM GPIO for encoder 1 channel A
  encoder1_pin_b: 17        # BCM GPIO for encoder 1 channel B
  encoder2_pin_a: 5         # BCM GPIO for encoder 2 channel A
  encoder2_pin_b: 6         # BCM GPIO for encoder 2 channel B

uart:
  lidar_port: /dev/serial0   # Raspberry Pi UART alias for pins 8 (TXD) and 10 (RXD)
  lidar_baudrate: 230400
```

## GPIO Pinout

### Encoder Pins (Raspberry Pi BCM numbering)
- **Motor 1 Encoder**: GPIO 4 (A), GPIO 17 (B)
- **Motor 2 Encoder**: GPIO 5 (A), GPIO 6 (B)

### IMU Pins
- **I2C**: SDA (GPIO 2), SCL (GPIO 3)
- **Address**: 0x68 (default for MPU6050/6500)

### LiDAR Pins
- **UART**: TXD GPIO 14 on pin 8, RXD GPIO 15 on pin 10
- **Device**: `/dev/serial0`
- **Baud**: 230400

## Odometry Integration

The wheel odometry node uses:
1. **Quadrature decoding** — reads encoder A and B channels to determine tick count and direction
2. **Exact arc integration** — integrates motion as a circular arc for accurate pose estimation
3. **Yaw normalization** — keeps heading bounded to $[-\pi, \pi]$
4. **Velocity computation** — publishes linear and angular velocities for UKF

## Tuning

### Covariances

Edit `params/ukf.yaml` to tune:
- `process_noise_covariance` — how much the filter trusts the motion model
- `odom0_config` and `imu0_config` — which measurements to fuse and how to weight them

### Encoder Noise

If encoder readings are noisy:
- Increase `encoder_resolution_m2` queue size
- Add filtering in the encoder callbacks
- Check GPIO pull-up resistor values

## Troubleshooting

### Node won't start
- Check GPIO permissions: `sudo usermod -a -G gpio $USER`
- Verify Python dependencies: `pip3 install RPi.GPIO rclpy nav_msgs`

### Odometry drifts
- Verify wheel radius and track width measurements
- Check encoder mounting and alignment
- Tune UKF covariances in `ukf.yaml`

### IMU not publishing
- Check I2C bus: `i2cdetect -y 1`
- Verify sensor address and wiring

## Next Steps

1. **Measure your robot**: wheel radius, track width, actual encoder resolution
2. **Update `measurements.yaml`** with real values
3. **Test wheelodom alone**: `ros2 topic echo /odom`
4. **Add IMU**: implement `imuodom.py` with your sensor driver
5. **Test LiDAR**: verify `ros2 topic echo /scan` on `/dev/serial0`
6. **Tune UKF**: adjust covariances based on real odometry and IMU data
