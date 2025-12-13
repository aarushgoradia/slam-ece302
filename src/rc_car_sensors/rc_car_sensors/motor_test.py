#!/usr/bin/env python3
"""
Hardware PWM Test Script for RC Car (Pi 5)

Requirements:
1. pip install rpi-hardware-pwm --break-system-packages
2. Add to /boot/firmware/config.txt: dtoverlay=pwm-2chan
3. Reboot

Hardware PWM Channels on Pi 5:
- PWM0 = GPIO 12 (Servo)
- PWM1 = GPIO 13 (Motor)
"""

from rpi_hardware_pwm import HardwarePWM
import time

# PWM Channels
SERVO_CHANNEL = 0  # GPIO 12
MOTOR_CHANNEL = 1  # GPIO 13
PWM_CHIP = 2       # Pi 5 uses chip 2

# Motor settings (inverted)
MOTOR_STOP = 100.0    # 100% = stopped
MOTOR_SLOW = 92.0     # 92% = slow
MOTOR_MEDIUM = 88.0   # 88% = medium
MOTOR_FAST = 85.0     # 85% = fast

# Servo settings
SERVO_LEFT = 5.0      # 5% = full left
SERVO_CENTER = 7.5    # 7.5% = center
SERVO_RIGHT = 10.0    # 10% = full right


def test_servo(servo):
    """Test servo positions."""
    print("\n=== SERVO TEST (Hardware PWM) ===")
    print("Testing servo - should be smooth with NO jitter!")
    
    print("  Center...")
    servo.change_duty_cycle(SERVO_CENTER)
    time.sleep(1)
    
    print("  Left...")
    servo.change_duty_cycle(SERVO_LEFT)
    time.sleep(1)
    
    print("  Center...")
    servo.change_duty_cycle(SERVO_CENTER)
    time.sleep(1)
    
    print("  Right...")
    servo.change_duty_cycle(SERVO_RIGHT)
    time.sleep(1)
    
    print("  Center...")
    servo.change_duty_cycle(SERVO_CENTER)
    time.sleep(1)
    
    print("  Sweep left to right...")
    for duty in range(50, 101, 5):  # 5.0% to 10.0%
        servo.change_duty_cycle(duty / 10.0)
        time.sleep(0.1)
    
    print("  Sweep right to left...")
    for duty in range(100, 49, -5):  # 10.0% to 5.0%
        servo.change_duty_cycle(duty / 10.0)
        time.sleep(0.1)
    
    servo.change_duty_cycle(SERVO_CENTER)
    print("Servo test complete!")


def test_motor(motor):
    """Test motor speeds."""
    print("\n=== MOTOR TEST (Hardware PWM) ===")
    print("WARNING: Wheels will spin! Lift the car!")
    
    input("Press Enter to start motor test (Ctrl+C to skip)...")
    
    print(f"  Stopped ({MOTOR_STOP}%)...")
    motor.change_duty_cycle(MOTOR_STOP)
    time.sleep(2)
    
    print(f"  Slow ({MOTOR_SLOW}%)...")
    motor.change_duty_cycle(MOTOR_SLOW)
    time.sleep(2)
    
    print(f"  Medium ({MOTOR_MEDIUM}%)...")
    motor.change_duty_cycle(MOTOR_MEDIUM)
    time.sleep(2)
    
    print(f"  Stopping...")
    motor.change_duty_cycle(MOTOR_STOP)
    time.sleep(1)
    
    print("Motor test complete!")


def test_combined(motor, servo):
    """Test motor and servo together."""
    print("\n=== COMBINED TEST ===")
    print("WARNING: Car will move!")
    
    input("Press Enter to start (Ctrl+C to skip)...")
    
    print("  Forward + straight...")
    servo.change_duty_cycle(SERVO_CENTER)
    motor.change_duty_cycle(MOTOR_SLOW)
    time.sleep(2)
    
    print("  Forward + left...")
    servo.change_duty_cycle(SERVO_LEFT)
    time.sleep(2)
    
    print("  Forward + right...")
    servo.change_duty_cycle(SERVO_RIGHT)
    time.sleep(2)
    
    print("  Stopping...")
    motor.change_duty_cycle(MOTOR_STOP)
    servo.change_duty_cycle(SERVO_CENTER)
    time.sleep(1)
    
    print("Combined test complete!")


def interactive_mode(motor, servo):
    """Interactive control."""
    print("\n=== INTERACTIVE MODE ===")
    print("Commands:")
    print("  w = forward slow")
    print("  W = forward medium") 
    print("  s = stop")
    print("  a = steer left")
    print("  d = steer right")
    print("  c = center steering")
    print("  q = quit")
    
    current_motor = MOTOR_STOP
    
    try:
        while True:
            cmd = input("Command: ").strip()
            
            if cmd == 'w':
                current_motor = MOTOR_SLOW
                print(f"  Motor: SLOW ({current_motor}%)")
            elif cmd == 'W':
                current_motor = MOTOR_MEDIUM
                print(f"  Motor: MEDIUM ({current_motor}%)")
            elif cmd == 's':
                current_motor = MOTOR_STOP
                print(f"  Motor: STOP ({current_motor}%)")
            elif cmd == 'a':
                servo.change_duty_cycle(SERVO_LEFT)
                print(f"  Servo: LEFT ({SERVO_LEFT}%)")
                continue
            elif cmd == 'd':
                servo.change_duty_cycle(SERVO_RIGHT)
                print(f"  Servo: RIGHT ({SERVO_RIGHT}%)")
                continue
            elif cmd == 'c':
                servo.change_duty_cycle(SERVO_CENTER)
                print(f"  Servo: CENTER ({SERVO_CENTER}%)")
                continue
            elif cmd == 'q':
                break
            else:
                print("  Unknown command")
                continue
            
            motor.change_duty_cycle(current_motor)
            
    except EOFError:
        pass
    
    motor.change_duty_cycle(MOTOR_STOP)
    servo.change_duty_cycle(SERVO_CENTER)


def manual_duty_test(motor, servo):
    """Manually set duty cycles."""
    print("\n=== MANUAL DUTY CYCLE TEST ===")
    print("Enter duty cycles directly for calibration")
    print("Format: m<duty> for motor, s<duty> for servo, q to quit")
    print("Example: m92.5 = motor at 92.5%, s7.5 = servo at 7.5%")
    print("")
    print("Motor is INVERTED: 100=stop, 85=fast")
    print("")
    
    try:
        while True:
            cmd = input("Enter command: ").strip()
            
            if cmd == 'q':
                break
            elif cmd.startswith('m'):
                try:
                    value = float(cmd[1:])
                    if 0 <= value <= 100:
                        motor.change_duty_cycle(value)
                        print(f"  Motor set to {value}%")
                    else:
                        print("  Value must be 0-100")
                except ValueError:
                    print("  Invalid number")
            elif cmd.startswith('s'):
                try:
                    value = float(cmd[1:])
                    if 0 <= value <= 100:
                        servo.change_duty_cycle(value)
                        print(f"  Servo set to {value}%")
                    else:
                        print("  Value must be 0-100")
                except ValueError:
                    print("  Invalid number")
            else:
                print("  Unknown command. Use m<duty>, s<duty>, or q")
                
    except EOFError:
        pass
    
    motor.change_duty_cycle(MOTOR_STOP)
    servo.change_duty_cycle(SERVO_CENTER)


def pwm_sweep_test(motor):
    """Sweep motor PWM to find working range."""
    print("\n=== PWM SWEEP TEST ===")
    print("Sweeping motor from 100% to 80%")
    print("WARNING: Motor will spin!")
    
    input("Press Enter to start sweep (Ctrl+C to skip)...")
    
    for duty in [100, 98, 96, 94, 92, 90, 88, 86, 84, 82, 80]:
        print(f"  Duty: {duty}%")
        motor.change_duty_cycle(duty)
        time.sleep(1.5)
    
    print("Stopping...")
    motor.change_duty_cycle(MOTOR_STOP)
    print("Sweep complete!")


def main():
    print("=" * 50)
    print("RC Car Hardware PWM Test (Pi 5)")
    print("=" * 50)
    print(f"Servo: PWM{SERVO_CHANNEL} (GPIO 12)")
    print(f"Motor: PWM{MOTOR_CHANNEL} (GPIO 13)")
    print(f"PWM Chip: {PWM_CHIP}")
    print("")
    print("Motor PWM is INVERTED:")
    print(f"  {MOTOR_STOP}% = stopped")
    print(f"  {MOTOR_SLOW}% = slow")
    print(f"  {MOTOR_MEDIUM}% = medium")
    print(f"  {MOTOR_FAST}% = fast")
    print("")
    
    try:
        # Initialize hardware PWM
        print("Initializing hardware PWM...")
        servo = HardwarePWM(pwm_channel=SERVO_CHANNEL, hz=50, chip=PWM_CHIP)
        motor = HardwarePWM(pwm_channel=MOTOR_CHANNEL, hz=50, chip=PWM_CHIP)
        
        # Start with safe values
        servo.start(SERVO_CENTER)
        motor.start(MOTOR_STOP)
        print("Hardware PWM initialized successfully!")
        print("")
        
    except Exception as e:
        print(f"ERROR: Failed to initialize hardware PWM: {e}")
        print("")
        print("Make sure you have:")
        print("1. Installed: pip install rpi-hardware-pwm --break-system-packages")
        print("2. Added to /boot/firmware/config.txt: dtoverlay=pwm-2chan")
        print("3. Rebooted the Pi")
        return
    
    try:
        print("Select test mode:")
        print("  1 = Servo only test")
        print("  2 = Motor only test")
        print("  3 = Combined test")
        print("  4 = Interactive mode")
        print("  5 = Manual duty cycle test")
        print("  6 = PWM sweep test")
        print("  q = Quit")
        print("")
        
        while True:
            choice = input("Choose (1-6 or q): ").strip()
            
            if choice == '1':
                test_servo(servo)
            elif choice == '2':
                test_motor(motor)
            elif choice == '3':
                test_combined(motor, servo)
            elif choice == '4':
                interactive_mode(motor, servo)
            elif choice == '5':
                manual_duty_test(motor, servo)
            elif choice == '6':
                pwm_sweep_test(motor)
            elif choice == 'q':
                break
            else:
                print("Invalid choice")
                
    except KeyboardInterrupt:
        print("\nInterrupted!")
    finally:
        print("\nStopping PWM...")
        motor.change_duty_cycle(MOTOR_STOP)
        servo.change_duty_cycle(SERVO_CENTER)
        time.sleep(0.5)
        motor.stop()
        servo.stop()
        print("Done!")


if __name__ == '__main__':
    main()