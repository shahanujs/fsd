import RPi.GPIO as GPIO
import socket
import json
import time
import threading
import math

# --- GPIO Setup (same pins as rpiteleop.py) ---
MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN = 17, 27, 18
MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN = 22, 23, 25
SERVO_PIN = 12

# Ultrasonic Sensors (HC-SR04)
SONAR_L_TRIG, SONAR_L_ECHO = 5, 26
SONAR_R_TRIG, SONAR_R_ECHO = 19, 13

# Motor Encoders (via level shifter 5V->3.3V)
# Phase B = C1, Phase A = C2 (per user wiring)
ENC_L_A, ENC_L_B = 16, 24   # Left:  Phase A = GPIO16, Phase B = GPIO24
ENC_R_A, ENC_R_B = 21, 20   # Right: Phase A = GPIO21, Phase B = GPIO20

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup([MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN,
            MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_EN], GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup([SONAR_L_TRIG, SONAR_R_TRIG], GPIO.OUT)
GPIO.setup([SONAR_L_ECHO, SONAR_R_ECHO], GPIO.IN)
GPIO.setup([ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)

pwm_a = GPIO.PWM(MOTOR_A_EN, 1000)
pwm_b = GPIO.PWM(MOTOR_B_EN, 1000)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)

pwm_a.start(0)
pwm_b.start(0)
servo_pwm.start(0)

# --- Encoder Class ---
class WheelEncoder:
    """
    Quadrature encoder reader using GPIO interrupts.
    Counts both rising edges on channel A for speed,
    uses channel B to determine direction.
    """
    def __init__(self, pin_a, pin_b, ppr=390, wheel_diam_cm=6.5, name=""):
        """
        pin_a: GPIO BCM pin for encoder Phase A
        pin_b: GPIO BCM pin for encoder Phase B
        ppr: Pulses Per Revolution (encoder ticks per full shaft rotation)
             Common values: 11 PPR motor * 34:1 gear = ~374, adjust to your motor
        wheel_diam_cm: Wheel diameter in cm for speed calculation
        """
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ppr = ppr
        self.wheel_circumference = math.pi * wheel_diam_cm  # cm per revolution
        self.name = name

        self.pulse_count = 0
        self.direction = 1  # 1 = forward, -1 = backward
        self.rpm = 0.0
        self.speed_cms = 0.0  # cm/s
        self._last_time = time.time()
        self._last_count = 0
        self._lock = threading.Lock()

        # Attach interrupt on Phase A rising edge
        GPIO.add_event_detect(pin_a, GPIO.RISING, callback=self._pulse_callback, bouncetime=1)

    def _pulse_callback(self, channel):
        """Called on every rising edge of Phase A"""
        # Read Phase B to determine direction
        b_state = GPIO.input(self.pin_b)
        with self._lock:
            if b_state:
                self.direction = 1   # forward
            else:
                self.direction = -1  # backward
            self.pulse_count += 1

    def update(self):
        """Call periodically to compute RPM and speed. Returns (rpm, speed_cm_s)"""
        now = time.time()
        with self._lock:
            count = self.pulse_count
            direction = self.direction
            self.pulse_count = 0
        
        dt = now - self._last_time
        self._last_time = now

        if dt > 0:
            revolutions = count / self.ppr
            self.rpm = (revolutions / dt) * 60.0
            self.speed_cms = (revolutions * self.wheel_circumference) / dt
            self.speed_cms *= direction
            self.rpm *= direction

        return self.rpm, self.speed_cms

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

# --- Create Encoder Instances ---
enc_left = WheelEncoder(ENC_L_A, ENC_L_B, ppr=390, wheel_diam_cm=6.5, name="Left")
enc_right = WheelEncoder(ENC_R_A, ENC_R_B, ppr=390, wheel_diam_cm=6.5, name="Right")

def encoder_thread():
    """Update encoder RPM/speed at ~20 Hz"""
    while sonar_running:
        enc_left.update()
        enc_right.update()
        time.sleep(0.05)

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
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", UDP_PORT))
sock.settimeout(0.5)

# Start sonar monitoring thread
t = threading.Thread(target=sonar_thread, daemon=True)
t.start()

# Start encoder monitoring thread
t_enc = threading.Thread(target=encoder_thread, daemon=True)
t_enc.start()

print(f"Cockpit Receiver Started! Listening on port {UDP_PORT}...")
print(f"Sonar: L(TRIG={SONAR_L_TRIG},ECHO={SONAR_L_ECHO}) R(TRIG={SONAR_R_TRIG},ECHO={SONAR_R_ECHO})")
print(f"Encoders: L(A={ENC_L_A},B={ENC_L_B}) R(A={ENC_R_A},B={ENC_R_B})")

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            cmd = json.loads(data.decode("utf-8"))

            throttle = float(cmd.get("t", 0))
            steer = float(cmd.get("s", 0))
            apply_controls(throttle, steer)

            # Send telemetry back with real distances + encoder data
            telemetry = {
                "dL": int(dist_L),
                "dR": int(dist_R),
                "rpmL": round(enc_left.rpm, 1),
                "rpmR": round(enc_right.rpm, 1),
                "spdL": round(enc_left.speed_cms, 1),
                "spdR": round(enc_right.speed_cms, 1)
            }
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
