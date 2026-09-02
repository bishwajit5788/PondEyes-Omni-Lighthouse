import paho.mqtt.client as mqtt
import json
import time
import math
import sys

# Instantiate the standard callback engine using the modern API mapping standard
client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)

try:
    client.connect("127.0.0.1", 1883, 60)
    print("[SIMULATOR] Connected to local broker. Initializing 10Hz network radar data...")
except Exception as e:
    print(f"[CRITICAL BROKER ERROR]: {e}")
    print("Ensure Mosquitto is active on your Mac via: brew services start mosquitto")
    sys.exit(1)

angle = 0
while True:
    try:
        # Calculate a smooth concentric tracking trajectory around your 5000,5000 center
        x_val = int(5000 + 2500 * math.cos(math.radians(angle)))
        y_val = int(5000 + 2500 * math.sin(math.radians(angle)))
        
        # Determine current sector face interval path
        current_sensor = int((angle % 360) // 60) + 1
        
        # Format identical schema payload structure matching the PondEyes specification
        mock_payload = {
            "system": "PondEyes Omni-Lighthouse (MOCK)",
            "timestamp_ms": int(time.time() * 1000),
            "rate_hz": 10,
            "total_active_targets": 1,
            "fused_targets": [
                {
                    "sensor": current_sensor,
                    "track_id": 1,
                    "x": x_val,
                    "y": y_val,
                    "speed": 22,
                    "distance": 2500,
                    "fused": (angle % 60 < 10), # Force localized seam fusion flags near the 60 deg seams
                    "source_mask": int(1 << (current_sensor - 1))
                }
            ]
        }
        
        client.publish("PondEyes/lighthouse/fused", json.dumps(mock_payload))
        angle = (angle + 2) % 360
        time.sleep(0.1) # Strict 100ms processing loop constraint (10Hz)
        
    except KeyboardInterrupt:
        print("\n[SIMULATOR] Transmission terminated cleanly.")
        break

client.disconnect()
