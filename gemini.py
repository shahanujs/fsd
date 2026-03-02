import pygame
import socket
import json
import time

PI_IP = "192.168.0.168" # Example: "192.168.1.50"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Pygame Setup
pygame.init()
screen = pygame.display.set_mode((300, 200))
pygame.display.set_caption("Laptop Teleop Controller")
clock = pygame.time.Clock()

STEERING_CENTER, STEERING_MAX, STEERING_MIN = 90, 130, 50
current_steering = STEERING_CENTER

print("Laptop Controller Started! Click the popup window to drive.")

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    
    # Emergency quit
    if keys[pygame.K_q] or keys[pygame.K_ESCAPE]:
        running = False

    # 1. Calculate Direction and Speed
    speed = 0
    direction = "forward"
    if keys[pygame.K_UP] or keys[pygame.K_w]:
        speed = 60
        direction = "forward"
    elif keys[pygame.K_DOWN] or keys[pygame.K_s]:
        speed = 60
        direction = "backward"

    # 2. Calculate Steering
    if keys[pygame.K_LEFT] or keys[pygame.K_a]:
        current_steering = max(STEERING_MIN, current_steering - 5)
    elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
        current_steering = min(STEERING_MAX, current_steering + 5)
    else:
        # Auto-center
        if current_steering > STEERING_CENTER:
            current_steering -= 10
        elif current_steering < STEERING_CENTER:
            current_steering += 10
            
        # Snap to center if close
        if abs(current_steering - STEERING_CENTER) < 10:
            current_steering = STEERING_CENTER

    # 3. Send to Pi
    packet = {
        "speed": speed,
        "direction": direction,
        "steering": current_steering
    }
    
    screen.fill((0, 100, 0) if pygame.mouse.get_focused() else (100, 0, 0))
    pygame.display.flip()
    
    sock.sendto(json.dumps(packet).encode("utf-8"), (PI_IP, UDP_PORT))
    
    clock.tick(50) # Send 50 packets per second

# Send final stop command before exiting
stop_packet = {"speed": 0, "direction": "forward", "steering": 90}
sock.sendto(json.dumps(stop_packet).encode("utf-8"), (PI_IP, UDP_PORT))
pygame.quit()
print("Controller closed.")
