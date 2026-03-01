import RPi.GPIO as GPIO
import socket
import json
import time

# --- GPIO Setup ---
MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN = 17, 27, 18
MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN = 22, 23, 25
SERVO_PIN = 12

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup([MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN, MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN], GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm_a = GPIO.PWM(MOTOR_A_EN, 1000)
pwm_b = GPIO.PWM(MOTOR_B_EN, 1000)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)

pwm_a.start(0)
pwm_b.start(0)
servo_pwm.start(0)

def apply_controls(speed, direction, steering):
    # Steering
    duty = steering / 18 + 2
    servo_pwm.ChangeDutyCycle(duty)
    
    # Motors
    if direction == "forward":
        GPIO.output(MOTOR_A_IN1, GPIO.HIGH); GPIO.output(MOTOR_A_IN2, GPIO.LOW)
        GPIO.output(MOTOR_B_IN1, GPIO.HIGH); GPIO.output(MOTOR_B_IN2, GPIO.LOW)
    elif direction == "backward":
        GPIO.output(MOTOR_A_IN1, GPIO.LOW); GPIO.output(MOTOR_A_IN2, GPIO.HIGH)
        GPIO.output(MOTOR_B_IN1, GPIO.LOW); GPIO.output(MOTOR_B_IN2, GPIO.HIGH)
    
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

# --- Network Setup ---
UDP_IP = "0.0.0.0"  # Listen on all available network interfaces
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.5) # If laptop disconnects, timeout and stop car!

print(f"Robot Receiver Started! Listening on port {UDP_PORT}...")

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            commands = json.loads(data.decode("utf-8"))
            apply_controls(commands["speed"], commands["direction"], commands["steering"])
        except socket.timeout:
            # Failsafe: Stop car if no signal received for 0.5 seconds
            apply_controls(0, "forward", 90)

except KeyboardInterrupt:
    print("\nShutting down robot...")
finally:
    apply_controls(0, "forward", 90)
    time.sleep(0.1)
    GPIO.cleanup()