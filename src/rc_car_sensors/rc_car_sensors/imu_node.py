#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import struct
import time


class ImuNode(Node):
    # BNO055 I2C address and registers
    BNO055_ADDRESS = 0x28
    OPR_MODE_REG = 0x3D
    UNIT_SEL_REG = 0x3B
    GYRO_DATA_REG = 0x14
    ACCEL_DATA_REG = 0x08
    
    # Operating modes
    CONFIG_MODE = 0x00
    IMU_MODE = 0x08  # Gyro + Accel (no magnetometer)

    def __init__(self):
        super().__init__('imu_node')
        
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('i2c_bus', 1)
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.i2c_bus = self.get_parameter('i2c_bus').value
        
        # Initialize I2C
        self.bus = smbus2.SMBus(self.i2c_bus)
        time.sleep(0.1)
        
        # Configure BNO055
        self._configure_sensor()
        
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.create_timer(1.0 / self.publish_rate, self.publish_imu)
        
        self.get_logger().info('BNO055 IMU node started (IMU mode, axes corrected)')
    
    def _configure_sensor(self):
        # Set to config mode
        self.bus.write_byte_data(self.BNO055_ADDRESS, self.OPR_MODE_REG, self.CONFIG_MODE)
        time.sleep(0.025)
        
        # Set units: m/s^2 for accel, rad/s for gyro
        self.bus.write_byte_data(self.BNO055_ADDRESS, self.UNIT_SEL_REG, 0x02)
        time.sleep(0.01)
        
        # Set to IMU mode (gyro + accel, no magnetometer fusion)
        self.bus.write_byte_data(self.BNO055_ADDRESS, self.OPR_MODE_REG, self.IMU_MODE)
        time.sleep(0.05)
        
        self.get_logger().info('BNO055 configured in IMU mode')
    
    def _read_gyro(self):
        data = self.bus.read_i2c_block_data(self.BNO055_ADDRESS, self.GYRO_DATA_REG, 6)
        x, y, z = struct.unpack('<hhh', bytes(data))
        scale = 1.0 / 900.0  # Convert to rad/s
        return (x * scale, y * scale, z * scale)
    
    def _read_accel(self):
        data = self.bus.read_i2c_block_data(self.BNO055_ADDRESS, self.ACCEL_DATA_REG, 6)
        x, y, z = struct.unpack('<hhh', bytes(data))
        scale = 1.0 / 100.0  # Convert to m/s^2
        return (x * scale, y * scale, z * scale)
    
    def _transform_axes(self, x, y, z):
        """
        Transform BNO055 axes to ROS convention.
        
        BNO055 mounted with:
          X+ pointing right (toward LiDAR)
          Y+ pointing forward
          Z+ pointing up
        
        ROS expects:
          X+ forward
          Y+ left
          Z+ up
        
        Transform:
          ROS_X = BNO_Y  (forward)
          ROS_Y = -BNO_X (left = negative right)
          ROS_Z = BNO_Z  (up)
        """
        ros_x = y
        ros_y = -x
        ros_z = z
        return ros_x, ros_y, ros_z
    
    def publish_imu(self):
        try:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            
            # No orientation - let Cartographer compute it
            msg.orientation.w = 1.0
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation_covariance[0] = -1.0  # Not provided
            
            # Gyroscope (raw, axes corrected)
            gx, gy, gz = self._read_gyro()
            gx, gy, gz = self._transform_axes(gx, gy, gz)
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            msg.angular_velocity_covariance[0] = 0.01
            msg.angular_velocity_covariance[4] = 0.01
            msg.angular_velocity_covariance[8] = 0.01
            
            # Accelerometer (raw, axes corrected)
            ax, ay, az = self._read_accel()
            ax, ay, az = self._transform_axes(ax, ay, az)
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            msg.linear_acceleration_covariance[0] = 0.1
            msg.linear_acceleration_covariance[4] = 0.1
            msg.linear_acceleration_covariance[8] = 0.1
            
            self.imu_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().warn(f'IMU read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()