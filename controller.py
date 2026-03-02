import pygame
import socket
import json
import threading
import io
import cv2
import numpy as np
import urllib.request
import math
import subprocess
import signal
import sys
import time
import atexit

class RobotCockpit:
    def __init__(self, robot_ip="192.168.0.168", robot_port=5005, web_port=8000, ssh_host="rpi"):
        # --- CONFIGURATION ---
        self.ROBOT_IP = robot_ip
        self.ROBOT_PORT = robot_port
        self.WEB_PORT = web_port
        self.SSH_HOST = ssh_host
        self.STREAM_URL = f"http://{robot_ip}:{web_port}/video_feed"
        
        # Pi-side script paths
        self.PI_RECEIVER = "/home/anuj/robot/drive_record/roboracer/cockpit_receiver.py"
        self.PI_VIDEO = "/home/anuj/robot/drive_record/roboracer/video_stream.py"
        
        # TUNING
        self.STEER_SPEED = 0.15
        self.CENTER_SPEED = 0.25
        self.MAX_EST_SPEED_CMS = 100.0
        
        # Speed limiter: caps max throttle output (like a speed dial)
        # 0.1 = 10% power, 0.5 = 50%, 1.0 = full power
        self.speed_limit = 0.5       # Default 50% — adjust with +/- keys
        self.SPEED_LIMIT_STEP = 0.05  # How much +/- changes per press
        
        # Throttle ramping (smooth acceleration)
        self.ACCEL_RATE = 0.02       # How fast throttle ramps up per frame (hold key)
        self.DECEL_RATE = 0.04       # How fast throttle ramps down on release (engine brake)
        self.BRAKE_RATE = 0.06       # How fast throttle ramps when braking (opposite dir)

        # STATE VARIABLES
        self.current_steer = 0.0
        self.dist_L = 999
        self.dist_R = 999
        self.rpm_L = 0.0
        self.rpm_R = 0.0
        self.speed_L = 0.0   # cm/s from encoder
        self.speed_R = 0.0   # cm/s from encoder
        self.throttle_cmd = 0.0      # Raw target (-1 to 1)
        self.throttle_actual = 0.0   # Smoothed actual throttle sent to car
        self.running = True
        self.current_frame = None
        self.video_running = True
        self.ssh_processes = []
        self.joystick = None
        self.dragging_slider = False
        self.slider_rect = None  # set during draw
        self.dragging_steer_slider = False
        self.steer_slider_rect = None
        self.steer_limit_deg = 32  # max steering angle in degrees (0-80), default ~40%
        self.dragging_dist_slider = False
        self.dist_slider_rect = None
        self.obstacle_dist_cm = 30  # stop if obstacle within this distance (cm)
        self.obstacle_blocked = False  # True when auto-stop is active
        self.obstacle_avoidance_on = False  # Toggle for obstacle avoidance
        self.oa_button_rect = None  # set during draw

        self._cleanup_done = False

        # --- SETUP MODULES ---
        self._start_pi_services()
        atexit.register(self._stop_pi_services)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        self._init_network()
        self._init_pygame()
        self._start_video_thread()

    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C and SIGTERM gracefully"""
        self.running = False

    def _start_pi_services(self):
        """Start receiver and video stream on Pi via SSH"""
        print("[LAUNCHER] Starting Pi services...")
        
        # Kill any old instances first
        subprocess.run(
            ["ssh", self.SSH_HOST,
             "sudo pkill -f cockpit_receiver.py; sudo pkill -f video_stream.py"],
            capture_output=True, timeout=10
        )
        time.sleep(1)
        
        # Start cockpit receiver (needs sudo for GPIO)
        # -tt forces TTY so remote process dies when SSH is killed
        print("[LAUNCHER] Starting cockpit_receiver.py on Pi...")
        p1 = subprocess.Popen(
            ["ssh", "-tt", self.SSH_HOST, f"sudo python3 {self.PI_RECEIVER}"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            stdin=subprocess.DEVNULL
        )
        self.ssh_processes.append(p1)
        
        # Start video stream
        print("[LAUNCHER] Starting video_stream.py on Pi...")
        p2 = subprocess.Popen(
            ["ssh", "-tt", self.SSH_HOST, f"python3 {self.PI_VIDEO}"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            stdin=subprocess.DEVNULL
        )
        self.ssh_processes.append(p2)
        
        # Give services time to start
        time.sleep(4)
        
        # Check if they're still running
        for i, p in enumerate(self.ssh_processes):
            if p.poll() is not None:
                stderr = p.stderr.read().decode() if p.stderr else ""
                print(f"[LAUNCHER] WARNING: Pi process {i} exited (code {p.returncode}): {stderr[:200]}")
            else:
                print(f"[LAUNCHER] Pi process {i} running (pid {p.pid})")
        
        print("[LAUNCHER] Pi services started.")

    def _stop_pi_services(self):
        """Stop all Pi services"""
        if self._cleanup_done:
            return
        self._cleanup_done = True
        print("[LAUNCHER] Stopping Pi services...")
        
        # First kill local SSH tunnels — with -tt, this sends SIGHUP to remote
        for p in self.ssh_processes:
            try:
                p.terminate()
                p.wait(timeout=2)
            except Exception:
                try:
                    p.kill()
                    p.wait(timeout=1)
                except Exception:
                    pass
        self.ssh_processes.clear()
        
        # Then explicitly kill any remaining remote processes
        kill_cmd = (
            "sudo pkill -9 -f cockpit_receiver.py; "
            "sudo pkill -9 -f video_stream.py; "
            "sudo pkill -9 -f 'python3.*video_stream'; "
            "sudo pkill -9 -f 'python3.*cockpit_receiver'; "
            "sudo fuser -k /dev/video0 2>/dev/null"
        )
        for _ in range(2):
            try:
                subprocess.run(
                    ["ssh", "-o", "ConnectTimeout=3", "-o", "StrictHostKeyChecking=no",
                     self.SSH_HOST, kill_cmd],
                    capture_output=True, timeout=5
                )
            except Exception:
                pass
            time.sleep(0.3)
        
        print("[LAUNCHER] Pi services stopped.")

    def _init_network(self):
        """Setup UDP Socket"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False) 

    def _init_pygame(self):
        """Setup Resizable Window and Fonts"""
        import os
        os.environ['SDL_VIDEO_ALLOW_SCREENCOMPOSER'] = '1'
        pygame.init()
        
        # Start as a normal resizable window
        self.SCREEN_W = 900
        self.SCREEN_H = 500
        self.screen = pygame.display.set_mode(
            (self.SCREEN_W, self.SCREEN_H), pygame.RESIZABLE
        )
        pygame.display.set_caption(f"FSD ROBOT COCKPIT - {self.ROBOT_IP}")
        self.clock = pygame.time.Clock()
        self.is_fullscreen = False
        
        # Init gamepad if available
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"[GAMEPAD] Found: {self.joystick.get_name()}")
        else:
            print("[INPUT] No gamepad found — using keyboard with smooth ramping")
        
        self._update_layout()

    def _update_layout(self):
        """Recompute all layout dimensions from current window size"""
        W, H = self.SCREEN_W, self.SCREEN_H
        
        # Panel width: ~25% of screen, min 200, max 320
        self.PANEL_W = max(200, min(320, int(W * 0.25)))
        self.VIDEO_W = W - self.PANEL_W - 20
        self.VIDEO_H = H - 20
        
        # Scale fonts relative to screen height
        scale = H / 500.0
        self.font_sm = pygame.font.SysFont("monospace", max(11, int(13 * scale)))
        self.font_md = pygame.font.SysFont("monospace", max(13, int(18 * scale)))
        self.font_lg = pygame.font.SysFont("monospace", max(18, int(26 * scale)))
        self.font_xl = pygame.font.SysFont("monospace", max(22, int(32 * scale)))
        self.font_rpm = pygame.font.SysFont("monospace", max(14, int(16 * scale)), bold=True)

    def _start_video_thread(self):
        """Starts the background video fetcher"""
        t = threading.Thread(target=self._video_loop)
        t.daemon = True
        t.start()

    def _video_loop(self):
        """Runs inside a thread to fetch MJPEG stream"""
        try:
            stream = urllib.request.urlopen(self.STREAM_URL)
            bytes_data = b''
            while self.video_running:
                bytes_data += stream.read(1024)
                a = bytes_data.find(b'\xff\xd8')
                b = bytes_data.find(b'\xff\xd9')
                
                if a != -1 and b != -1:
                    jpg = bytes_data[a:b+2]
                    bytes_data = bytes_data[b+2:]
                    
                    try:
                        nparr = np.frombuffer(jpg, np.uint8)
                        frame_cv = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                        
                        if frame_cv is not None:
                            # Convert BGR to RGB & Transpose for Pygame
                            frame_rgb = cv2.cvtColor(frame_cv, cv2.COLOR_BGR2RGB)
                            frame_rgb = np.transpose(frame_rgb, (1, 0, 2))
                            frame_surface = pygame.surfarray.make_surface(frame_rgb)
                            
                            # Store raw surface, scale at draw time for resize
                            self.current_frame = frame_surface
                    except Exception:
                        pass
        except Exception as e:
            print(f"Video Stream Error: {e}")

    def _handle_input(self):
        """Process Keyboard & Events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.VIDEORESIZE:
                if not self.is_fullscreen:
                    self.SCREEN_W, self.SCREEN_H = event.w, event.h
                    self.screen = pygame.display.set_mode(
                        (self.SCREEN_W, self.SCREEN_H), pygame.RESIZABLE
                    )
                    self._update_layout()
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.oa_button_rect and self.oa_button_rect.collidepoint(event.pos):
                    self.obstacle_avoidance_on = not self.obstacle_avoidance_on
                elif self.slider_rect and self.slider_rect.collidepoint(event.pos):
                    self.dragging_slider = True
                    self._update_slider_from_mouse(event.pos[0])
                elif self.steer_slider_rect and self.steer_slider_rect.collidepoint(event.pos):
                    self.dragging_steer_slider = True
                    self._update_steer_slider_from_mouse(event.pos[0])
                elif self.dist_slider_rect and self.dist_slider_rect.collidepoint(event.pos):
                    self.dragging_dist_slider = True
                    self._update_dist_slider_from_mouse(event.pos[0])
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self.dragging_slider = False
                self.dragging_steer_slider = False
                self.dragging_dist_slider = False
            elif event.type == pygame.MOUSEMOTION:
                if self.dragging_slider and self.slider_rect:
                    self._update_slider_from_mouse(event.pos[0])
                if self.dragging_steer_slider and self.steer_slider_rect:
                    self._update_steer_slider_from_mouse(event.pos[0])
                if self.dragging_dist_slider and self.dist_slider_rect:
                    self._update_dist_slider_from_mouse(event.pos[0])
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_F11:
                    # Toggle fullscreen
                    self.is_fullscreen = not self.is_fullscreen
                    if self.is_fullscreen:
                        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
                        di = pygame.display.Info()
                        self.SCREEN_W, self.SCREEN_H = di.current_w, di.current_h
                    else:
                        self.SCREEN_W, self.SCREEN_H = 900, 500
                        self.screen = pygame.display.set_mode(
                            (self.SCREEN_W, self.SCREEN_H), pygame.RESIZABLE
                        )
                    self._update_layout()
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
                    self.speed_limit = min(1.0, self.speed_limit + self.SPEED_LIMIT_STEP)
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    self.speed_limit = max(0.01, self.speed_limit - self.SPEED_LIMIT_STEP)

        # Also track actual surface size each frame (some WMs skip VIDEORESIZE)
        if not self.is_fullscreen:
            actual_w, actual_h = self.screen.get_size()
            if actual_w != self.SCREEN_W or actual_h != self.SCREEN_H:
                self.SCREEN_W, self.SCREEN_H = actual_w, actual_h
                self._update_layout()

        keys = pygame.key.get_pressed()
        
        # ESC to quit
        if keys[pygame.K_ESCAPE]:
            self.running = False
            return
        
        # --- THROTTLE (smooth ramping) ---
        if self.joystick:
            # Gamepad: analog triggers for true pressure control
            # Axis 5 = right trigger (gas), Axis 4 = left trigger (brake)
            # Triggers go from -1 (released) to 1 (fully pressed)
            try:
                gas = (self.joystick.get_axis(5) + 1) / 2.0      # 0 to 1
                brake = (self.joystick.get_axis(4) + 1) / 2.0    # 0 to 1
                self.throttle_cmd = gas - brake
                # Gamepad steering (left stick X axis)
                stick_x = self.joystick.get_axis(0)
                if abs(stick_x) > 0.1:  # deadzone
                    self.current_steer = stick_x
                else:
                    # Auto center
                    if self.current_steer > 0:
                        self.current_steer = max(0, self.current_steer - self.CENTER_SPEED)
                    elif self.current_steer < 0:
                        self.current_steer = min(0, self.current_steer + self.CENTER_SPEED)
            except Exception:
                pass
        else:
            # Keyboard: set target, actual ramps toward it
            self.throttle_cmd = 0.0
            if keys[pygame.K_UP]: self.throttle_cmd = 1.0
            if keys[pygame.K_DOWN]: self.throttle_cmd = -1.0
        
        # Smooth ramp throttle_actual toward throttle_cmd
        diff = self.throttle_cmd - self.throttle_actual
        if abs(diff) < 0.01:
            self.throttle_actual = self.throttle_cmd
        elif self.throttle_cmd == 0:
            # Releasing — coast down
            if self.throttle_actual > 0:
                self.throttle_actual = max(0, self.throttle_actual - self.DECEL_RATE)
            else:
                self.throttle_actual = min(0, self.throttle_actual + self.DECEL_RATE)
        elif (self.throttle_cmd > 0 and self.throttle_actual < 0) or \
             (self.throttle_cmd < 0 and self.throttle_actual > 0):
            # Braking (reversing direction) — faster ramp
            self.throttle_actual += math.copysign(self.BRAKE_RATE, diff)
        else:
            # Accelerating in same direction — smooth curve
            # Use ease-in: ramp faster at low speed, slower near max
            speed_factor = 1.0 - abs(self.throttle_actual) * 0.5
            self.throttle_actual += math.copysign(self.ACCEL_RATE * speed_factor, diff)
        
        # Clamp throttle within speed limit
        self.throttle_actual = max(-self.speed_limit, min(self.speed_limit, self.throttle_actual))

        # Obstacle avoidance: block forward throttle if obstacle detected
        if self.obstacle_avoidance_on:
            min_dist = min(self.dist_L, self.dist_R)
            if min_dist < self.obstacle_dist_cm and self.throttle_actual > 0:
                self.throttle_actual = 0.0
                self.throttle_cmd = 0.0
                self.obstacle_blocked = True
            else:
                self.obstacle_blocked = False
        else:
            self.obstacle_blocked = False

        # Steering (keyboard, only if no gamepad)
        if not self.joystick:
            if keys[pygame.K_LEFT]: 
                self.current_steer -= self.STEER_SPEED
            elif keys[pygame.K_RIGHT]: 
                self.current_steer += self.STEER_SPEED
            else:
                # Auto Center
                if self.current_steer > 0: 
                    self.current_steer -= self.CENTER_SPEED
                    if self.current_steer < 0: self.current_steer = 0
                elif self.current_steer < 0: 
                    self.current_steer += self.CENTER_SPEED
                    if self.current_steer > 0: self.current_steer = 0
        
        # Clamp steer within angle limit
        max_steer = self.steer_limit_deg / 80.0
        self.current_steer = max(-max_steer, min(max_steer, self.current_steer))

    def _update_slider_from_mouse(self, mouse_x):
        """Update speed limit based on mouse X position on the slider"""
        if self.slider_rect:
            rel_x = mouse_x - self.slider_rect.x
            pct = rel_x / self.slider_rect.width
            self.speed_limit = max(0.01, min(1.0, pct))

    def _update_steer_slider_from_mouse(self, mouse_x):
        """Update steering angle limit based on mouse X position"""
        if self.steer_slider_rect:
            rel_x = mouse_x - self.steer_slider_rect.x
            pct = rel_x / self.steer_slider_rect.width
            self.steer_limit_deg = max(0, min(80, int(pct * 80)))

    def _update_dist_slider_from_mouse(self, mouse_x):
        """Update obstacle distance threshold from mouse position (5-200 cm)"""
        if self.dist_slider_rect:
            rel_x = mouse_x - self.dist_slider_rect.x
            pct = rel_x / self.dist_slider_rect.width
            self.obstacle_dist_cm = max(5, min(200, int(5 + pct * 195)))

    def _update_network(self):
        """Send Command & Receive Telemetry"""
        # SEND (use smoothed throttle_actual, not raw throttle_cmd)
        cmd = {"t": round(self.throttle_actual, 3), "s": round(self.current_steer, 3)}
        self.sock.sendto(json.dumps(cmd).encode('utf-8'), (self.ROBOT_IP, self.ROBOT_PORT))

        # RECEIVE
        try:
            data, _ = self.sock.recvfrom(1024)
            info = json.loads(data.decode('utf-8'))
            self.dist_L = info.get("dL", 999)
            self.dist_R = info.get("dR", 999)
            self.rpm_L = info.get("rpmL", 0.0)
            self.rpm_R = info.get("rpmR", 0.0)
            self.speed_L = info.get("spdL", 0.0)
            self.speed_R = info.get("spdR", 0.0)
        except BlockingIOError:
            pass
        except Exception:
            pass

    def _draw_speedometer(self, x, y, radius):
        """Draws the Analog Gauge with real encoder speed"""
        # Use average of both wheel speeds (real encoder data)
        enc_speed = (self.speed_L + self.speed_R) / 2.0
        # Fallback to throttle estimate if encoders read zero and throttle is active
        if abs(enc_speed) < 0.1 and abs(self.throttle_actual) > 0.01:
            speed = self.throttle_actual * self.MAX_EST_SPEED_CMS
        else:
            speed = enc_speed
        
        is_reverse = speed < 0
        abs_speed = abs(speed)
        
        # Arc
        rect = pygame.Rect(x - radius, y - radius, radius * 2, radius * 2)
        arc_color = (100, 100, 100) if not is_reverse else (100, 50, 50)
        pygame.draw.arc(self.screen, arc_color, rect, 0, 3.14, 10)
        
        # Speed limit arc (yellow zone)
        limit_angle = 3.14 * (1 - self.speed_limit)
        pygame.draw.arc(self.screen, (80, 80, 0), rect, 0, limit_angle, 6)
        
        # Needle (uses absolute speed for position)
        pct = min(abs_speed / self.MAX_EST_SPEED_CMS, 1.0)
        angle = 180 - (pct * 180)
        rad = math.radians(angle)
        
        end_x = x + (radius - 10) * math.cos(rad)
        end_y = y - (radius - 10) * math.sin(rad)
        
        color = (0, 255, 255) if not is_reverse else (255, 100, 100)
        pygame.draw.line(self.screen, color, (x, y), (end_x, end_y), 4)
        pygame.draw.circle(self.screen, color, (x, y), 5)
        
        # Scale labels
        self.screen.blit(self.font_sm.render("0", 1, (150, 150, 150)), (x - radius, y + 10))
        self.screen.blit(self.font_sm.render("100", 1, (150, 150, 150)), (x + radius - 30, y + 10))
        
        # Speed value (always positive, like a real car speedometer)
        spd_text = self.font_lg.render(f"{int(abs_speed)}", 1, (255, 255, 255))
        self.screen.blit(spd_text, (x - spd_text.get_width()//2, y - 40))
        self.screen.blit(self.font_sm.render("cm/s", 1, (150,150,150)), (x - 15, y - 10))
        
        # Drive/Reverse gear indicator (Tesla-style D/R)
        gear = "R" if is_reverse else "D"
        gear_color = (255, 80, 80) if is_reverse else (0, 220, 0)
        gear_text = self.font_rpm.render(gear, 1, gear_color)
        self.screen.blit(gear_text, (x + radius - 20, y - 15))
        
        # Speed limit indicator
        limit_pct = int(self.speed_limit * 100)
        limit_color = (255, 255, 0) if limit_pct < 60 else (0, 255, 0)
        lim_text = self.font_sm.render(f"LIMIT: {limit_pct}%  [+/-]", 1, limit_color)
        self.screen.blit(lim_text, (x - lim_text.get_width()//2, y + 20))

        # RPM display — always positive (magnitude only), bold
        rpm_l = abs(int(self.rpm_L))
        rpm_r = abs(int(self.rpm_R))
        rpm_text = self.font_rpm.render(f"RPM  L:{rpm_l}  R:{rpm_r}", 1, (0, 230, 230))
        self.screen.blit(rpm_text, (x - rpm_text.get_width()//2, y + 35))

    def _draw_interface(self):
        """Main Drawing Loop - Fully Dynamic Proportional Layout"""
        W, H = self.SCREEN_W, self.SCREEN_H
        self.screen.fill((30, 30, 30))

        # 1. VIDEO (fills left side)
        vw, vh = self.VIDEO_W, self.VIDEO_H
        pygame.draw.rect(self.screen, (0, 0, 0), (10, 10, vw, vh))
        if self.current_frame:
            scaled = pygame.transform.scale(self.current_frame, (vw, vh))
            self.screen.blit(scaled, (10, 10))
        else:
            txt = self.font_lg.render("WAITING FOR VIDEO...", 1, (255, 255, 255))
            self.screen.blit(txt, (10 + vw // 2 - txt.get_width() // 2, H // 2))

        # 2. DASHBOARD (right panel - proportional layout)
        panel_x = W - self.PANEL_W
        pw = self.PANEL_W - 20
        half_pw = pw // 2 - 5
        gap = max(4, int(H * 0.006))  # proportional gap
        slider_h = max(32, int(H * 0.055))  # slider height scales with window
        bar_h_track = max(8, int(H * 0.015))  # thin track bar

        # Current Y cursor - flows top to bottom
        cy = 10

        # --- Sonar Boxes ---
        sonar_h = max(45, int(H * 0.09))
        col_L = (0, 255, 0) if self.dist_L >= 60 else (255, 255, 0) if self.dist_L >= 30 else (255, 0, 0)
        pygame.draw.rect(self.screen, (50, 50, 50), (panel_x, cy, half_pw, sonar_h))
        self.screen.blit(self.font_sm.render("L-DIST", 1, (200, 200, 200)), (panel_x + 5, cy + 3))
        self.screen.blit(self.font_lg.render(f"{self.dist_L}", 1, col_L), (panel_x + 5, cy + sonar_h // 2))

        col_R = (0, 255, 0) if self.dist_R >= 60 else (255, 255, 0) if self.dist_R >= 30 else (255, 0, 0)
        pygame.draw.rect(self.screen, (50, 50, 50), (panel_x + half_pw + 10, cy, half_pw, sonar_h))
        self.screen.blit(self.font_sm.render("R-DIST", 1, (200, 200, 200)), (panel_x + half_pw + 15, cy + 3))
        self.screen.blit(self.font_lg.render(f"{self.dist_R}", 1, col_R), (panel_x + half_pw + 15, cy + sonar_h // 2))
        cy += sonar_h + gap

        # --- Speedometer ---
        gauge_radius = max(35, min(80, int(H * 0.12)))
        gauge_cx = panel_x + pw // 2
        gauge_y = cy + gauge_radius + 10
        self._draw_speedometer(gauge_cx, gauge_y, gauge_radius)
        cy = gauge_y + gauge_radius + max(20, int(H * 0.03))

        # --- Steering Bar ---
        steer_box_h = max(30, int(H * 0.06))
        pygame.draw.rect(self.screen, (50, 50, 50), (panel_x, cy, pw, steer_box_h))
        self.screen.blit(self.font_sm.render("STEERING", 1, (200, 200, 200)), (gauge_cx - 35, cy + 2))
        bar_y = cy + steer_box_h - bar_h_track - 5
        bar_w = pw - 20
        bar_cx = gauge_cx
        pygame.draw.rect(self.screen, (100, 100, 100), (panel_x + 10, bar_y, bar_w, bar_h_track))
        bar_half = bar_w // 2
        if self.current_steer < 0:
            bw = abs(self.current_steer) * bar_half
            pygame.draw.rect(self.screen, (255, 0, 0), (bar_cx - bw, bar_y, bw, bar_h_track))
        elif self.current_steer > 0:
            bw = abs(self.current_steer) * bar_half
            pygame.draw.rect(self.screen, (0, 255, 0), (bar_cx, bar_y, bw, bar_h_track))
        pygame.draw.line(self.screen, (255, 255, 255), (bar_cx, bar_y - 2), (bar_cx, bar_y + bar_h_track + 2), 2)
        cy += steer_box_h + gap

        # --- Sliders (MAX SPEED, MAX STEER, STOP DIST) ---
        slider_x = panel_x + 10
        slider_w = pw - 20
        track_h = max(12, int(slider_h * 0.45))

        # Helper to draw a slider
        def draw_slider(sy, label, value_text, fill_pct, fill_color):
            rect = pygame.Rect(slider_x, sy, slider_w, slider_h)
            pygame.draw.rect(self.screen, (50, 50, 50), rect)
            self.screen.blit(self.font_sm.render(label, 1, (200, 200, 200)), (slider_x + 2, sy + 1))
            track_y = sy + slider_h - track_h - 2
            fill_w = int(fill_pct * slider_w)
            pygame.draw.rect(self.screen, fill_color, (slider_x, track_y, fill_w, track_h))
            pygame.draw.rect(self.screen, (100, 100, 100), (slider_x, track_y, slider_w, track_h), 1)
            pygame.draw.rect(self.screen, (255, 255, 255), (slider_x + fill_w - 3, track_y - 2, 6, track_h + 4))
            vt = self.font_sm.render(value_text, 1, (255, 255, 255))
            self.screen.blit(vt, (slider_x + slider_w - vt.get_width() - 2, sy + 1))
            return rect

        # MAX SPEED slider
        limit_pct = int(self.speed_limit * 100)
        sp_color = (0, 200, 0) if limit_pct <= 30 else (255, 255, 0) if limit_pct <= 60 else (255, 80, 80)
        self.slider_rect = draw_slider(cy, "MAX SPEED", f"{limit_pct}%", self.speed_limit, sp_color)
        cy += slider_h + gap

        # MAX STEER slider
        st_color = (0, 200, 0) if self.steer_limit_deg <= 25 else (0, 180, 255) if self.steer_limit_deg <= 50 else (255, 165, 0)
        self.steer_slider_rect = draw_slider(cy, "MAX STEER", f"{self.steer_limit_deg}\u00b0", self.steer_limit_deg / 80.0, st_color)
        cy += slider_h + gap

        # STOP DIST slider
        dist_pct = (self.obstacle_dist_cm - 5) / 195.0
        dt_color = (0, 200, 0) if self.obstacle_dist_cm <= 30 else (255, 200, 0) if self.obstacle_dist_cm <= 80 else (255, 80, 80)
        self.dist_slider_rect = draw_slider(cy, "STOP DIST", f"{self.obstacle_dist_cm}cm", dist_pct, dt_color)
        cy += slider_h + gap

        # --- Obstacle Avoidance Toggle Button ---
        oa_btn_h = max(22, int(H * 0.035))
        self.oa_button_rect = pygame.Rect(slider_x, cy, slider_w, oa_btn_h)
        if self.obstacle_avoidance_on:
            btn_color = (0, 150, 0)
            btn_label = "OBSTACLE AVOID: ON"
        else:
            btn_color = (150, 0, 0)
            btn_label = "OBSTACLE AVOID: OFF"
        pygame.draw.rect(self.screen, btn_color, self.oa_button_rect)
        pygame.draw.rect(self.screen, (200, 200, 200), self.oa_button_rect, 1)
        btn_text = self.font_sm.render(btn_label, 1, (255, 255, 255))
        self.screen.blit(btn_text, (slider_x + slider_w // 2 - btn_text.get_width() // 2,
                                    cy + oa_btn_h // 2 - btn_text.get_height() // 2))
        cy += oa_btn_h + gap

        # Obstacle blocked warning overlay on video
        if self.obstacle_blocked:
            warn_surf = self.font_lg.render("OBSTACLE - STOPPED", 1, (255, 0, 0))
            blink = int(time.time() * 4) % 2
            if blink:
                wx = 10 + vw // 2 - warn_surf.get_width() // 2
                wy = 10 + vh - 50
                pygame.draw.rect(self.screen, (0, 0, 0), (wx - 10, wy - 5, warn_surf.get_width() + 20, warn_surf.get_height() + 10))
                self.screen.blit(warn_surf, (wx, wy))

        # Hints at bottom
        hint = "F11:fullscreen | +/-:speed | ESC:quit"
        self.screen.blit(self.font_sm.render(hint, 1, (100, 100, 100)), (panel_x + 5, H - 20))

        pygame.display.flip()

    def run(self):
        """Main Application Loop"""
        print("--- COCKPIT ONLINE ---")
        print("Press Ctrl+C or close window to stop everything.")
        try:
            while self.running:
                self._handle_input()
                self._update_network()
                self._draw_interface()
                self.clock.tick(30)
        except KeyboardInterrupt:
            pass
        finally:
            print("\n--- SHUTTING DOWN ---")
            self.video_running = False
            self._stop_pi_services()
            pygame.quit()
            print("--- GOODBYE ---")

if __name__ == "__main__":
    cockpit = RobotCockpit(robot_ip="192.168.0.168", ssh_host="rpi")
    cockpit.run()
