import RPi.GPIO as GPIO
import socket
import json
import time
import threading

# --- GPIO Setup (same pins as rpiteleop.py) ---
MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN = 17, 27, 18
MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN = 22, 23, 25
SERVO_PIN = 12

# Ultrasonic Sensors (HC-SR04)
SONAR_L_TRIG, SONAR_L_ECHO = 5, 26
SONAR_R_TRIG, SONAR_R_ECHO = 19, 13

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup([MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN,
            MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN], GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup([SONAR_L_TRIG, SONAR_R_TRIG], GPIO.OUT)
GPIO.setup([SONAR_L_ECHO, SONAR_R_ECHO], GPIO.IN)

pwm_a = GPIO.PWM(MOTOR_A_EN, 1000)
pwm_b = GPIO.PWM(MOTOR_B_EN, 1000)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)

pwm_a.start(0)
pwm_b.start(0)
servo_pwm.start(0)

# --- Ultrasonic Distance Reading ---
dist_L = 999.0
dist_R = 999.0
sonar_running = True

def measure_distance(trig, echo):
    """Measure distance in cm using HC-SR04"""
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start = time.time()
    timeout = start + 0.04  # 40ms timeout (~680cm max)

    while GPIO.input(echo) == 0:
        start = time.time()
        if start > timeout:
            return 999.0
    while GPIO.input(echo) == 1:
        stop = time.time()
        if stop > timeout:
            return 999.0

    return ((stop - start) * 34300) / 2  # speed of sound / 2

def sonar_thread():
    """Continuously read both ultrasonic sensors"""
    global dist_L, dist_R, sonar_running
    while sonar_running:
        dist_L = measure_distance(SONAR_L_TRIG, SONAR_L_ECHO)
        time.sleep(0.01)
        dist_R = measure_distance(SONAR_R_TRIG, SONAR_R_ECHO)
        time.sleep(0.06)  # ~12 Hz total update rate

def apply_controls(throttle, steer):
    """
    throttle: -1.0 to 1.0 (positive=forward, negative=backward)
    steer:    -1.0 to 1.0 (negative=left, positive=right)
    """
    # --- Motors ---
    speed = abs(throttle) * 100  # duty cycle 0-100

    if throttle > 0:
        GPIO.output(MOTOR_A_IN1, GPIO.HIGH); GPIO.output(MOTOR_A_IN2, GPIO.LOW)
        GPIO.output(MOTOR_B_IN1, GPIO.HIGH); GPIO.output(MOTOR_B_IN2, GPIO.LOW)
    elif throttle < 0:
        GPIO.output(MOTOR_A_IN1, GPIO.LOW); GPIO.output(MOTOR_A_IN2, GPIO.HIGH)
        GPIO.output(MOTOR_B_IN1, GPIO.LOW); GPIO.output(MOTOR_B_IN2, GPIO.HIGH)
    else:
        GPIO.output(MOTOR_A_IN1, GPIO.LOW); GPIO.output(MOTOR_A_IN2, GPIO.LOW)
        GPIO.output(MOTOR_B_IN1, GPIO.LOW); GPIO.output(MOTOR_B_IN2, GPIO.LOW)

    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

    # --- Servo Steering ---
    # Map steer [-1.0, 1.0] -> angle [50, 130] with center 90
    angle = 90 + (steer * 40)
    angle = max(50, min(130, angle))
    duty = angle / 18 + 2
    servo_pwm.ChangeDutyCycle(duty)

def stop():
    apply_controls(0, 0)

# --- Network Setup ---
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
sock.settimeout(0.5)

# Start sonar monitoring thread
t = threading.Thread(target=sonar_thread, daemon=True)
t.start()

print(f"Cockpit Receiver Started! Listening on port {UDP_PORT}...")
print(f"Sonar: L(TRIG={SONAR_L_TRIG},ECHO={SONAR_L_ECHO}) R(TRIG={SONAR_R_TRIG},ECHO={SONAR_R_ECHO})")

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            cmd = json.loads(data.decode("utf-8"))

            throttle = float(cmd.get("t", 0))
            steer = float(cmd.get("s", 0))
            apply_controls(throttle, steer)

            # Send telemetry back with real distances
            telemetry = {"dL": int(dist_L), "dR": int(dist_R)}
            sock.sendto(json.dumps(telemetry).encode("utf-8"), addr)

        except socket.timeout:
            stop()

except KeyboardInterrupt:
    print("\nShutting down...")
finally:
    sonar_running = False
    stop()
    time.sleep(0.1)
    servo_pwm.stop()
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
    print("GPIO cleaned up. Goodbye.")
