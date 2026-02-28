"""
Author: anuj
Company: anuj corporation
Date: YYYY-MM-DD
Time: HH:MM:SS
Purpose:
    Control a Raspberry Pi based robot using keyboard teleoperation,
    steering servo control, motor speed/direction control, and optional
    camera recording with action logging for driving sessions.
"""

import cv2
import time
import RPi.GPIO as GPIO
from pathlib import Path
import sys
import termios
import select
import threading


class TeleopController:
    """Robot teleop controller with keyboard input for steering and speed control"""
    
    # GPIO Pin Configuration
    MOTOR_A_IN1 = 17
    MOTOR_A_IN2 = 27
    MOTOR_A_EN = 18
    MOTOR_B_IN1 = 22
    MOTOR_B_IN2 = 23
    MOTOR_B_EN = 25
    SERVO_PIN = 12
    
    # Control Parameters
    MAX_SPEED = 60
    STEERING_CENTER = 90
    STEERING_MIN = 75  # Center - 15°
    STEERING_MAX = 105  # Center + 15°
    STEERING_STEP = 5
    KEY_HOLD_TIMEOUT = 0.5  # 500ms - very long timeout for simultaneous keys
    
    # Video Configuration
    VIDEO_WIDTH = 640
    VIDEO_HEIGHT = 480
    VIDEO_FPS = 20
    
    def __init__(self):
        """Initialize the controller"""
        self.quit_flag = False
        self.recording = False
        self.old_settings = None
        
        # State variables
        self.current_speed = 0
        self.current_steering = self.STEERING_CENTER
        self.frame_count = 0
        self.start_time = None
        
        # Key state tracking
        self.key_last_seen = {
            'up': -1,
            'down': -1,
            'left': -1,
            'right': -1
        }
        
        # Separate timeouts for different key types
        self.MOVEMENT_KEY_TIMEOUT = 0.3  # 300ms - enough time for both keys to register
        self.STEERING_KEY_TIMEOUT = 0.35  # 350ms - enough time for steering with movement
        
        # Camera and video objects
        self.cap = None
        self.out = None
        self.action_log = None
        self.video_path = None
        self.action_path = None
        
        # PWM objects
        self.pwm_a = None
        self.pwm_b = None
        self.servo_pwm = None
        
    def setup_gpio(self):
        """Configure GPIO pins and PWM"""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        
        # Setup motor pins
        motor_pins = [
            self.MOTOR_A_IN1, self.MOTOR_A_IN2, self.MOTOR_A_EN,
            self.MOTOR_B_IN1, self.MOTOR_B_IN2, self.MOTOR_B_EN
        ]
        GPIO.setup(motor_pins, GPIO.OUT)
        
        # Setup PWM for motors
        self.pwm_a = GPIO.PWM(self.MOTOR_A_EN, 1000)
        self.pwm_a.start(0)
        self.pwm_b = GPIO.PWM(self.MOTOR_B_EN, 1000)
        self.pwm_b.start(0)
        
        # Setup servo pin and PWM
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(self.SERVO_PIN, 50)
        self.servo_pwm.start(0)
        
    def setup_camera(self):
        """Initialize camera and video recording"""
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.VIDEO_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.VIDEO_HEIGHT)
        
        # Create data directory
        data_dir = Path('data')
        data_dir.mkdir(exist_ok=True)
        
        # Generate timestamp-based filenames
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.video_path = data_dir / f"drive_{ts}.avi"
        self.action_path = data_dir / f"actions_{ts}.txt"
        
        # Setup video writer
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(str(self.video_path), fourcc, self.VIDEO_FPS, 
                                   (self.VIDEO_WIDTH, self.VIDEO_HEIGHT))
        
        # Setup action log
        self.action_log = open(self.action_path, 'w')
        self.start_time = time.time()
        
    def set_speed(self, speed):
        """Set motor speed (0-100%)"""
        new_speed = max(0, min(100, speed))
        if new_speed != self.current_speed:
            self.current_speed = new_speed
            self.pwm_a.ChangeDutyCycle(self.current_speed)
            self.pwm_b.ChangeDutyCycle(self.current_speed)
    
    def set_direction(self, forward=True):
        """Set motor direction: forward or backward"""
        if forward:
            GPIO.output(self.MOTOR_A_IN1, GPIO.HIGH)
            GPIO.output(self.MOTOR_A_IN2, GPIO.LOW)
            GPIO.output(self.MOTOR_B_IN1, GPIO.HIGH)
            GPIO.output(self.MOTOR_B_IN2, GPIO.LOW)
        else:
            GPIO.output(self.MOTOR_A_IN1, GPIO.LOW)
            GPIO.output(self.MOTOR_A_IN2, GPIO.HIGH)
            GPIO.output(self.MOTOR_B_IN1, GPIO.LOW)
            GPIO.output(self.MOTOR_B_IN2, GPIO.HIGH)
    
    def set_steering(self, angle):
        """Set servo steering angle (within safe limits)"""
        new_steering = max(self.STEERING_MIN, min(self.STEERING_MAX, angle))
        if abs(new_steering - self.current_steering) >= 1:
            self.current_steering = new_steering
            duty = self.current_steering / 18 + 2
            self.servo_pwm.ChangeDutyCycle(duty)
    
    def stop(self):
        """Stop all movement and return steering to center"""
        self.set_speed(0)
        self.set_steering(self.STEERING_CENTER)
    
    def read_keys(self):
        """Background thread: Read keyboard input and update key states"""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            
            # Configure terminal: no echo, raw input
            new_settings = termios.tcgetattr(sys.stdin)
            new_settings[3] = new_settings[3] & ~termios.ICANON & ~termios.ECHO
            new_settings[6][termios.VMIN] = 1   # Blocking read - wait for at least 1 char
            new_settings[6][termios.VTIME] = 0  # No timeout
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings)
            
            while not self.quit_flag:
                try:
                    # Blocking read - will wait for input
                    ch = sys.stdin.read(1)
                    if not ch:
                        continue
                    
                    # Check for Ctrl+C
                    try:
                        if ord(ch) == 3:
                            self.quit_flag = True
                            return
                    except:
                        pass
                    
                    if ch.lower() == 'q':
                        self.quit_flag = True
                        return
                    elif ch.lower() == 'r':
                        self.recording = not self.recording
                        status = "REC ON" if self.recording else "REC OFF"
                        sys.stderr.write(f"\r{status}     \n")
                        sys.stderr.flush()
                    elif ch == '\x1b':  # ESC - read full escape sequence
                        # Read [ (blocking)
                        bracket = sys.stdin.read(1)
                        # Read key code (blocking)
                        key_char = sys.stdin.read(1)
                        
                        # Update key timestamp immediately
                        current_time = time.time()
                        if bracket == '[' and key_char:
                            if key_char == 'A':  # UP
                                self.key_last_seen['up'] = current_time
                            elif key_char == 'B':  # DOWN
                                self.key_last_seen['down'] = current_time
                            elif key_char == 'C':  # RIGHT
                                self.key_last_seen['right'] = current_time
                            elif key_char == 'D':  # LEFT
                                self.key_last_seen['left'] = current_time
                except Exception as e:
                    pass
        except Exception as e:
            pass
        finally:
            if self.old_settings:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                except:
                    pass
    
    def update_controls(self):
        """Update motor and steering based on current key states"""
        current_time = time.time()
        
        # Determine which keys are currently "held" using separate timeouts
        # Movement keys (UP/DOWN) have shorter timeout for immediate stop
        key_up = (current_time - self.key_last_seen['up']) < self.MOVEMENT_KEY_TIMEOUT
        key_down = (current_time - self.key_last_seen['down']) < self.MOVEMENT_KEY_TIMEOUT
        
        # Steering keys (LEFT/RIGHT) have slightly longer timeout for smooth turning
        key_left = (current_time - self.key_last_seen['left']) < self.STEERING_KEY_TIMEOUT
        key_right = (current_time - self.key_last_seen['right']) < self.STEERING_KEY_TIMEOUT
        
        # Handle forward/backward movement (mutually exclusive)
        if key_up:
            self.set_direction(True)
            self.set_speed(self.MAX_SPEED)
        elif key_down:
            self.set_direction(False)
            self.set_speed(self.MAX_SPEED)
        else:
            self.set_speed(0)
        
        # Handle steering (independent of movement - both keys cannot turn both ways)
        if key_left and not key_right:
            # Only left pressed - turn left
            self.set_steering(self.current_steering - self.STEERING_STEP)
        elif key_right and not key_left:
            # Only right pressed - turn right
            self.set_steering(self.current_steering + self.STEERING_STEP)
        else:
            # No steering input or both pressed (center) - auto-return to center
            if self.current_steering > self.STEERING_CENTER:
                self.set_steering(self.current_steering - self.STEERING_STEP)
            elif self.current_steering < self.STEERING_CENTER:
                self.set_steering(self.current_steering + self.STEERING_STEP)
    
    def process_frame(self):
        """Capture frame from camera and optionally record"""
        ret, frame = self.cap.read()
        if ret and self.recording:
            self.out.write(frame)
            self.frame_count += 1
            
            # Log action data
            ts_now = time.time() - self.start_time
            self.action_log.write(f"{ts_now:.3f},{self.current_steering:.1f},{self.current_speed:.1f}\n")
    
    def print_instructions(self):
        """Print control instructions"""
        print("=" * 50)
        print("Robot Teleop Controller - Ready to Drive!")
        print("=" * 50)
        print("Control (Hold arrow keys steady):")
        print("  ↑ UP    : forward (hold to move)")
        print("  ↓ DOWN  : reverse (hold to move)")
        print("  ← LEFT  : turn left (auto-centers)")
        print("  → RIGHT : turn right (auto-centers)")
        print("  R       : toggle record")
        print("  Q       : quit")
        print("=" * 50)
        print()
    
    def cleanup(self):
        """Clean up resources"""
        self.quit_flag = True
        time.sleep(0.1)
        
        # Release camera and video
        if self.cap:
            self.cap.release()
        if self.out:
            self.out.release()
        if self.action_log:
            self.action_log.close()
        
        # Stop motors and steering
        self.stop()
        
        # Cleanup GPIO
        GPIO.cleanup()
        
        # Restore terminal settings
        if self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except:
                pass
        
        # Print summary
        print("\n" + "=" * 50)
        print("Session Complete!")
        print(f"Frames recorded: {self.frame_count}")
        print(f"Video saved: {self.video_path}")
        print(f"Actions log: {self.action_path}")
        print("=" * 50)
    
    def run(self):
        """Main control loop"""
        try:
            # Initialize all systems
            self.setup_gpio()
            self.setup_camera()
            self.print_instructions()
            
            # Start keyboard input thread
            key_thread = threading.Thread(target=self.read_keys, daemon=True)
            key_thread.start()
            
            # Give thread time to set up terminal
            time.sleep(0.2)
            
            # Main control loop - ultra fast for responsive control
            while not self.quit_flag:
                self.update_controls()
                self.process_frame()
                time.sleep(0.003)  # ~330 FPS
        
        except KeyboardInterrupt:
            print("\nCtrl+C received")
            self.quit_flag = True
        except Exception as e:
            print(f"Error: {e}")
            self.quit_flag = True
        finally:
            self.cleanup()


def main():
    """Entry point"""
    controller = TeleopController()
    controller.run()


if __name__ == "__main__":
    main()