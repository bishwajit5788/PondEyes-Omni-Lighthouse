import pygame
import paho.mqtt.client as mqtt
import json
import math
import sys

# Screen Setup Constraints
SCREEN_SIZE = 800
ARENA_SIZE_MM = 10000
SCALE = SCREEN_SIZE / ARENA_SIZE_MM # Scales 10,000 mm area cleanly to 800 pixels

pygame.init()
screen = pygame.display.set_mode((SCREEN_SIZE, SCREEN_SIZE))
pygame.display.set_caption("PondEyes Omni-Lighthouse - Live 360° Tracking Canvas")
clock = pygame.time.Clock()

# Volatile global array to store target frames
live_targets = []

# MQTT Data Frame Handler Callback
def on_message(client, userdata, msg):
    global live_targets
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        if "fused_targets" in payload:
            live_targets = payload["fused_targets"]
    except Exception as e:
        pass

# Instantiate the standard callback engine using the modern API mapping standard
mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.on_message = on_message

try:
    mqtt_client.connect("127.0.0.1", 1883, 60)
    mqtt_client.subscribe("PondEyes/lighthouse/fused")
    mqtt_client.loop_start()
    print("[SUCCESS] Dashboard linked to local broker ports. Rendering canvas...")
except Exception as e:
    print(f"[FATAL NETWORK ERROR]: {e}")
    sys.exit(1)

# Application Main Window Render Loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear screen to deep dark-mode slate grid color
    screen.fill((15, 22, 30))

    center_px = int(5000 * SCALE)

    # 1. Render the 60-degree reference boundary seam overlay split lines
    # Maps directly to [30, 90, 150, 210, 270, 330] degree seam rays for visual audit
    for i in range(6):
        angle_deg = 30 + (i * 60)
        rad = math.radians(angle_deg)
        end_x = center_px + int(4500 * SCALE * math.sin(rad))
        end_y = center_px - int(4500 * SCALE * math.cos(rad))
        pygame.draw.line(screen, (255, 42, 42), (center_px, center_px), (end_x, end_y), 1)

    # 2. Render the physical core central hardware lighthouse diameter body
    pygame.draw.circle(screen, (88, 166, 255), (center_px, center_px), int(65 * SCALE) + 2)

    # 3. Dynamic target track coordinates rendering
    for target in live_targets:
        raw_x = target.get("x", 5000)
        raw_y = target.get("y", 5000)
        
        # Translate global mm values into relative screen Cartesian coordinate points
        px_x = int(raw_x * SCALE)
        px_y = int((ARENA_SIZE_MM - raw_y) * SCALE) # Invert Y for PyGame screen layout orientation
        
        # Change color matrix dynamically if the target is fused near boundary seams
        node_color = (51, 255, 102) if target.get("fused", False) else (255, 153, 51)
        
        # Plot active trajectory nodes
        pygame.draw.circle(screen, node_color, (px_x, px_y), 9)
        
        # Render tracking information metadata labels next to avatars
        font = pygame.font.SysFont("Arial", 14)
        info_text = font.render(f"ID:{target.get('track_id', 0)} (S{target.get('sensor', 0)})", True, (255, 255, 255))
        screen.blit(info_text, (px_x + 14, px_y - 7))

    pygame.display.flip()
    clock.tick(30) # Enforce a steady 30 FPS redraw refresh cap

mqtt_client.loop_stop()
pygame.quit()
sys.exit()
