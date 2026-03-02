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
        self.steer_limit_deg = 80  # max steering angle in degrees (0-80)
        self.dragging_dist_slider = False
        self.dist_slider_rect = None
        self.obstacle_dist_cm = 30  # stop if obstacle within this distance (cm)
        self.obstacle_blocked = False  # True when auto-stop is active

        # --- SETUP MODULES ---
        self._start_pi_services()
        self._init_network()
        self._init_pygame()
        self._start_video_thread()

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
        print("[LAUNCHER] Starting cockpit_receiver.py on Pi...")
        p1 = subprocess.Popen(
            ["ssh", self.SSH_HOST, f"sudo python3 {self.PI_RECEIVER}"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        self.ssh_processes.append(p1)
        
        # Start video stream
        print("[LAUNCHER] Starting video_stream.py on Pi...")
        p2 = subprocess.Popen(
            ["ssh", self.SSH_HOST, f"python3 {self.PI_VIDEO}"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
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
        print("[LAUNCHER] Stopping Pi services...")
        
        # Kill remote processes (sudo needed to kill the sudo receiver)
        try:
            subprocess.run(
                ["ssh", self.SSH_HOST,
                 "sudo pkill -f cockpit_receiver.py; sudo pkill -f video_stream.py"],
                capture_output=True, timeout=10
            )
        except Exception as e:
            print(f"[LAUNCHER] Warning: remote kill failed: {e}")
        
        # Kill local SSH tunnels
        for p in self.ssh_processes:
            try:
                p.terminate()
                p.wait(timeout=3)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
        
        self.ssh_processes.clear()
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
                if self.slider_rect and self.slider_rect.collidepoint(event.pos):
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
        min_dist = min(self.dist_L, self.dist_R)
        if min_dist < self.obstacle_dist_cm and self.throttle_actual > 0:
            self.throttle_actual = 0.0
            self.throttle_cmd = 0.0
            self.obstacle_blocked = True
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
        except BlockingIOError:
            pass
        except Exception:
            pass

    def _draw_speedometer(self, x, y, radius):
        """Draws the Analog Gauge with smooth speed"""
        speed = self.throttle_actual * self.MAX_EST_SPEED_CMS
        
        # Arc
        rect = pygame.Rect(x - radius, y - radius, radius * 2, radius * 2)
        pygame.draw.arc(self.screen, (100, 100, 100), rect, 0, 3.14, 10)
        
        # Speed limit arc (yellow zone)
        limit_angle = 3.14 * (1 - self.speed_limit)
        pygame.draw.arc(self.screen, (80, 80, 0), rect, 0, limit_angle, 6)
        
        # Needle
        pct = speed / self.MAX_EST_SPEED_CMS
        angle = 180 - (pct * 180)
        rad = math.radians(angle)
        
        end_x = x + (radius - 10) * math.cos(rad)
        end_y = y - (radius - 10) * math.sin(rad)
        
        color = (0, 255, 255) if speed >= 0 else (255, 100, 100)
        pygame.draw.line(self.screen, color, (x, y), (end_x, end_y), 4)
        pygame.draw.circle(self.screen, color, (x, y), 5)
        
        # Text
        self.screen.blit(self.font_sm.render("0", 1, (150, 150, 150)), (x - radius, y + 10))
        self.screen.blit(self.font_sm.render("100", 1, (150, 150, 150)), (x + radius - 30, y + 10))
        
        spd_text = self.font_lg.render(f"{abs(int(speed))}", 1, (255, 255, 255))
        self.screen.blit(spd_text, (x - spd_text.get_width()//2, y - 40))
        self.screen.blit(self.font_sm.render("cm/s", 1, (150,150,150)), (x - 15, y - 10))
        
        # Speed limit indicator
        limit_pct = int(self.speed_limit * 100)
        limit_color = (255, 255, 0) if limit_pct < 60 else (0, 255, 0)
        lim_text = self.font_sm.render(f"LIMIT: {limit_pct}%  [+/-]", 1, limit_color)
        self.screen.blit(lim_text, (x - lim_text.get_width()//2, y + 20))

    def _draw_interface(self):
        """Main Drawing Loop - Fully Dynamic Layout"""
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

        # 2. DASHBOARD (right panel - all positions relative to panel)
        panel_x = W - self.PANEL_W
        pw = self.PANEL_W - 20
        panel_h = H - 20  # usable panel height
        half_pw = pw // 2 - 5
        margin = 10
        
        # --- Distribute widgets vertically ---
        # Sonar boxes: 16% of panel height
        sonar_h = max(60, int(panel_h * 0.14))
        sonar_y = 10
        
        col_L = (0, 255, 0) if self.dist_L >= 60 else (255, 255, 0) if self.dist_L >= 30 else (255, 0, 0)
        pygame.draw.rect(self.screen, (50, 50, 50), (panel_x, sonar_y, half_pw, sonar_h))
        self.screen.blit(self.font_md.render("L-DIST", 1, (200, 200, 200)), (panel_x + 10, sonar_y + 8))
        self.screen.blit(self.font_xl.render(f"{self.dist_L}", 1, col_L), (panel_x + 10, sonar_y + sonar_h // 2))

        col_R = (0, 255, 0) if self.dist_R >= 60 else (255, 255, 0) if self.dist_R >= 30 else (255, 0, 0)
        pygame.draw.rect(self.screen, (50, 50, 50), (panel_x + half_pw + 10, sonar_y, half_pw, sonar_h))
        self.screen.blit(self.font_md.render("R-DIST", 1, (200, 200, 200)), (panel_x + half_pw + 20, sonar_y + 8))
        self.screen.blit(self.font_xl.render(f"{self.dist_R}", 1, col_R), (panel_x + half_pw + 20, sonar_y + sonar_h // 2))

        # Speedometer: centered in middle ~50% of panel
        gauge_radius = max(40, min(90, int(panel_h * 0.18)))
        gauge_cx = panel_x + pw // 2
        gauge_y = sonar_y + sonar_h + margin + gauge_radius + 20
        self._draw_speedometer(gauge_cx, gauge_y, gauge_radius)

        # Steering bar: below speedometer
        steer_y = gauge_y + gauge_radius + 30
        steer_h = max(45, int(panel_h * 0.10))
        pygame.draw.rect(self.screen, (50, 50, 50), (panel_x, steer_y, pw, steer_h))
        self.screen.blit(self.font_md.render("STEERING", 1, (200, 200, 200)),
                         (gauge_cx - 45, steer_y + 5))
        
        bar_cx = gauge_cx
        bar_y = steer_y + steer_h - 20
        bar_w = pw - 20
        pygame.draw.rect(self.screen, (100, 100, 100), (panel_x + 10, bar_y, bar_w, 12))
        
        bar_half = bar_w // 2
        if self.current_steer < 0:
            bw = abs(self.current_steer) * bar_half
            pygame.draw.rect(self.screen, (255, 0, 0), (bar_cx - bw, bar_y, bw, 12))
        elif self.current_steer > 0:
            bw = abs(self.current_steer) * bar_half
            pygame.draw.rect(self.screen, (0, 255, 0), (bar_cx, bar_y, bw, 12))
            
        pygame.draw.line(self.screen, (255, 255, 255), (bar_cx, bar_y - 2), (bar_cx, bar_y + 14), 2)

        # Speed Limit Slider (clickable/draggable)
        slider_y = bar_y + 30
        slider_w = pw - 20
        slider_h = 40
        slider_x = panel_x + 10
        self.slider_rect = pygame.Rect(slider_x, slider_y, slider_w, slider_h)
        
        # Background box
        pygame.draw.rect(self.screen, (50, 50, 50), self.slider_rect)
        self.screen.blit(self.font_sm.render("MAX SPEED", 1, (200, 200, 200)), (slider_x + 2, slider_y + 2))
        
        # Filled bar showing current limit
        fill_w = int(self.speed_limit * slider_w)
        limit_pct = int(self.speed_limit * 100)
        if limit_pct <= 30:
            fill_color = (0, 200, 0)
        elif limit_pct <= 60:
            fill_color = (255, 255, 0)
        else:
            fill_color = (255, 80, 80)
        pygame.draw.rect(self.screen, fill_color, (slider_x, slider_y + 18, fill_w, 18))
        
        # Track outline
        pygame.draw.rect(self.screen, (100, 100, 100), (slider_x, slider_y + 18, slider_w, 18), 1)
        
        # Thumb handle
        thumb_x = slider_x + fill_w
        pygame.draw.rect(self.screen, (255, 255, 255), (thumb_x - 3, slider_y + 16, 6, 22))
        
        # Percentage text
        pct_text = self.font_md.render(f"{limit_pct}%", 1, (255, 255, 255))
        self.screen.blit(pct_text, (slider_x + slider_w - pct_text.get_width() - 2, slider_y + 1))

        # Steering Angle Limit Slider
        steer_sl_y = slider_y + slider_h + 10
        steer_sl_w = slider_w
        steer_sl_h = 40
        steer_sl_x = slider_x
        self.steer_slider_rect = pygame.Rect(steer_sl_x, steer_sl_y, steer_sl_w, steer_sl_h)

        # Background box
        pygame.draw.rect(self.screen, (50, 50, 50), self.steer_slider_rect)
        self.screen.blit(self.font_sm.render("MAX STEER", 1, (200, 200, 200)), (steer_sl_x + 2, steer_sl_y + 2))

        # Filled bar
        steer_pct = self.steer_limit_deg / 80.0
        steer_fill_w = int(steer_pct * steer_sl_w)
        if self.steer_limit_deg <= 25:
            steer_color = (0, 200, 0)
        elif self.steer_limit_deg <= 50:
            steer_color = (0, 180, 255)
        else:
            steer_color = (255, 165, 0)
        pygame.draw.rect(self.screen, steer_color, (steer_sl_x, steer_sl_y + 18, steer_fill_w, 18))

        # Track outline
        pygame.draw.rect(self.screen, (100, 100, 100), (steer_sl_x, steer_sl_y + 18, steer_sl_w, 18), 1)

        # Thumb handle
        st_thumb_x = steer_sl_x + steer_fill_w
        pygame.draw.rect(self.screen, (255, 255, 255), (st_thumb_x - 3, steer_sl_y + 16, 6, 22))

        # Degree text
        deg_text = self.font_md.render(f"{self.steer_limit_deg}\u00b0", 1, (255, 255, 255))
        self.screen.blit(deg_text, (steer_sl_x + steer_sl_w - deg_text.get_width() - 2, steer_sl_y + 1))

        # Obstacle Distance Threshold Slider
        dist_sl_y = steer_sl_y + steer_sl_h + 10
        dist_sl_w = steer_sl_w
        dist_sl_h = 40
        dist_sl_x = slider_x
        self.dist_slider_rect = pygame.Rect(dist_sl_x, dist_sl_y, dist_sl_w, dist_sl_h)

        # Background box
        pygame.draw.rect(self.screen, (50, 50, 50), self.dist_slider_rect)
        self.screen.blit(self.font_sm.render("STOP DIST", 1, (200, 200, 200)), (dist_sl_x + 2, dist_sl_y + 2))

        # Filled bar (5-200 cm range)
        dist_pct = (self.obstacle_dist_cm - 5) / 195.0
        dist_fill_w = int(dist_pct * dist_sl_w)
        if self.obstacle_dist_cm <= 30:
            dist_color = (0, 200, 0)
        elif self.obstacle_dist_cm <= 80:
            dist_color = (255, 200, 0)
        else:
            dist_color = (255, 80, 80)
        pygame.draw.rect(self.screen, dist_color, (dist_sl_x, dist_sl_y + 18, dist_fill_w, 18))

        # Track outline
        pygame.draw.rect(self.screen, (100, 100, 100), (dist_sl_x, dist_sl_y + 18, dist_sl_w, 18), 1)

        # Thumb handle
        d_thumb_x = dist_sl_x + dist_fill_w
        pygame.draw.rect(self.screen, (255, 255, 255), (d_thumb_x - 3, dist_sl_y + 16, 6, 22))

        # Distance text
        dist_txt = self.font_md.render(f"{self.obstacle_dist_cm}cm", 1, (255, 255, 255))
        self.screen.blit(dist_txt, (dist_sl_x + dist_sl_w - dist_txt.get_width() - 2, dist_sl_y + 1))

        # Obstacle blocked warning overlay on video
        if self.obstacle_blocked:
            warn_surf = self.font_lg.render("OBSTACLE - STOPPED", 1, (255, 0, 0))
            blink = int(time.time() * 4) % 2  # blink at 2Hz
            if blink:
                wx = 10 + vw // 2 - warn_surf.get_width() // 2
                wy = 10 + vh - 50
                pygame.draw.rect(self.screen, (0, 0, 0), (wx - 10, wy - 5, warn_surf.get_width() + 20, warn_surf.get_height() + 10))
                self.screen.blit(warn_surf, (wx, wy))

        # Hints at bottom
        hint = "F11:fullscreen | +/-:speed | ESC:quit"
        self.screen.blit(self.font_sm.render(hint, 1, (100, 100, 100)), (panel_x + 5, H - 25))

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
