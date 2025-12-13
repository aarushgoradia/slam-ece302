#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        # Parameters - all floats for consistency
        self.declare_parameter('motor_pin', 13)
        self.declare_parameter('servo_pin', 12)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('motor_min_duty', 85.0)   # 85% = max speed (inverted)
        self.declare_parameter('motor_max_duty', 100.0)  # 100% = stopped (inverted)
        self.declare_parameter('servo_center_duty', 7.5)
        self.declare_parameter('servo_range_duty', 2.5)
        self.declare_parameter('timeout', 0.5)
        
        self.motor_pin = self.get_parameter('motor_pin').value
        self.servo_pin = self.get_parameter('servo_pin').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.motor_min_duty = float(self.get_parameter('motor_min_duty').value)
        self.motor_max_duty = float(self.get_parameter('motor_max_duty').value)
        self.servo_center_duty = float(self.get_parameter('servo_center_duty').value)
        self.servo_range_duty = float(self.get_parameter('servo_range_duty').value)
        self.timeout = self.get_parameter('timeout').value
        
        # Track current PWM values
        self.current_motor_duty = self.motor_max_duty
        self.current_servo_duty = self.servo_center_duty
        
        # Open GPIO chip (Pi 5 uses gpiochip4)
        try:
            self.chip = lgpio.gpiochip_open(4)  # Pi 5
            self.get_logger().info('Opened gpiochip4 (Pi 5)')
        except:
            self.chip = lgpio.gpiochip_open(0)  # Pi 4 fallback
            self.get_logger().info('Opened gpiochip0 (Pi 4 fallback)')
        
        # Set pins as output
        lgpio.gpio_claim_output(self.chip, self.motor_pin)
        lgpio.gpio_claim_output(self.chip, self.servo_pin)
        
        # PWM frequency
        self.motor_freq = 50   # 50Hz for ESC
        self.servo_freq = 50   # 50Hz for servo
        
        # Start PWM - stopped state
        lgpio.tx_pwm(self.chip, self.motor_pin, self.motor_freq, self.motor_max_duty)
        lgpio.tx_pwm(self.chip, self.servo_pin, self.servo_freq, self.servo_center_duty)
        
        self.last_cmd_time = self.get_clock().now()
        self.cmd_count = 0
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Safety timer
        self.create_timer(0.1, self.safety_check)
        
        # Status timer - print PWM values every second
        self.create_timer(1.0, self.print_status)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Motor controller started (lgpio)')
        self.get_logger().info(f'  Motor pin: GPIO {self.motor_pin}')
        self.get_logger().info(f'  Servo pin: GPIO {self.servo_pin}')
        self.get_logger().info(f'  Motor: {self.motor_max_duty}% = STOP, {self.motor_min_duty}% = FAST')
        self.get_logger().info(f'  Servo: {self.servo_center_duty}% = CENTER')
        self.get_logger().info('=' * 50)
    
    def print_status(self):
        """Print current PWM status every second."""
        motor_status = "STOP" if self.current_motor_duty >= 95 else "MOVING"
        self.get_logger().info(
            f'[PWM] Motor: {self.current_motor_duty:.1f}% ({motor_status}) | '
            f'Servo: {self.current_servo_duty:.1f}% | '
            f'Cmds: {self.cmd_count}'
        )
    
    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.cmd_count += 1
        
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Motor control (inverted)
        if linear_x <= 0:
            motor_duty = self.motor_max_duty
        else:
            speed = min(linear_x, self.max_speed)
            speed_ratio = speed / self.max_speed
            motor_duty = self.motor_max_duty - (speed_ratio * (self.motor_max_duty - self.motor_min_duty))
        
        # Servo control
        steering = max(-self.max_steering_angle, min(self.max_steering_angle, angular_z))
        steering_ratio = steering / self.max_steering_angle
        servo_duty = self.servo_center_duty - (steering_ratio * self.servo_range_duty)
        
        # Store current values
        self.current_motor_duty = motor_duty
        self.current_servo_duty = servo_duty
        
        # Apply PWM
        lgpio.tx_pwm(self.chip, self.motor_pin, self.motor_freq, motor_duty)
        lgpio.tx_pwm(self.chip, self.servo_pin, self.servo_freq, servo_duty)
        
        # Log every command
        self.get_logger().info(
            f'[CMD] lin={linear_x:.2f} ang={angular_z:.2f} -> Motor={motor_duty:.1f}% Servo={servo_duty:.1f}%'
        )
    
    def safety_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.timeout:
            if self.current_motor_duty != self.motor_max_duty:
                self.get_logger().warn('[SAFETY] Timeout! Stopping motor.')
                lgpio.tx_pwm(self.chip, self.motor_pin, self.motor_freq, self.motor_max_duty)
                self.current_motor_duty = self.motor_max_duty
    
    def stop(self):
        lgpio.tx_pwm(self.chip, self.motor_pin, self.motor_freq, self.motor_max_duty)
        lgpio.tx_pwm(self.chip, self.servo_pin, self.servo_freq, self.servo_center_duty)
        self.current_motor_duty = self.motor_max_duty
        self.current_servo_duty = self.servo_center_duty
    
    def destroy_node(self):
        self.get_logger().info('Shutting down motor controller...')
        self.stop()
        lgpio.gpiochip_close(self.chip)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()