#!/usr/bin/env python3

import cv2
import numpy as np
import os
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import time
import sys
import vl53l0x


# ============================================================
# USER CONFIGURATION SECTION
# ============================================================

# Grid setup
GRID_SIZE = 7
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# HSV mask for green (tuned for outdoor daylight)
HSV_LOWER = np.array([30, 60, 40])
HSV_UPPER = np.array([85, 255, 255])

# Detection thresholds
MIN_CONTOUR_AREA = 250
ORB_FEATURES = 200
GOOD_MATCH_COUNT = 15
MATCH_DIST_THRESH = 50

# Servo configuration
SERVO_PIN_BASE = 17
SERVO_PIN_ARM = 18
SERVO_FREQ = 50
BASE_RANGE = (30, 150)   # Base servo angle range (left ↔ right)
ARM_RANGE = (50, 120)    # Arm servo angle range (front ↔ back)

# Relay (Cutter Motor)
RELAY_PIN = 27           # GPIO pin controlling the relay
CUT_DURATION = 1.0       # Cutter on-time in seconds

# Reference weed image folder
REF_FOLDER = "weeds"

# ============================================================
# INITIALIZATION FUNCTIONS
# ============================================================

def setup_gpio():
    """Setup GPIO pins for servos and relay."""
    if GPIO is None:
        return None, None

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN_BASE, GPIO.OUT)
    GPIO.setup(SERVO_PIN_ARM, GPIO.OUT)
    GPIO.setup(RELAY_PIN, GPIO.OUT)

    # PWM for servos
    pwm_base = GPIO.PWM(SERVO_PIN_BASE, SERVO_FREQ)
    pwm_arm = GPIO.PWM(SERVO_PIN_ARM, SERVO_FREQ)
    pwm_base.start(0)
    pwm_arm.start(0)

    # Ensure cutter is OFF at start
    GPIO.output(RELAY_PIN, GPIO.LOW)
    return pwm_base, pwm_arm


def angle_to_duty(angle):
    """Convert servo angle to PWM duty cycle."""
    return 2 + (angle / 18.0)


def set_servo(pwm, angle):
    """Set servo angle safely."""
    if pwm is None:
        return
    duty = angle_to_duty(angle)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.25)
    pwm.ChangeDutyCycle(0)


def move_to_grid(pwm_base, pwm_arm, row, col, depth):
    """Move servo arms toward a grid cell based on row & col."""
    base_angle = np.interp(col, [0, GRID_SIZE - 1], BASE_RANGE)
    arm_angle  = np.interp(row, [0, GRID_SIZE - 1], ARM_RANGE)

    print(f"→ Moving to cell ({row},{col}) | depth={depth:.0f} mm | base={base_angle:.1f}°, arm={arm_angle:.1f}°")
    set_servo(pwm_base, base_angle)
    set_servo(pwm_arm, arm_angle)


def activate_cutter():
    if GPIO is None:
        print("Cutter relay ON")
        time.sleep(CUT_DURATION)
        print("Cutter relay OFF")
        return

    GPIO.output(RELAY_PIN, GPIO.HIGH)
    print("🌀 Cutter ON")
    time.sleep(CUT_DURATION)
    GPIO.output(RELAY_PIN, GPIO.LOW)
    print("✂️  Cutter OFF")


def read_lidar():
    
    try:
        sensor = vl53l0x.VL53L0X(i2c_bus=1)
        sensor.open()
        sensor.start_ranging(vl53l0x.VL53L0X_BETTER_ACCURACY_MODE)
        dist = sensor.get_distance()
        sensor.stop_ranging()
        return float(dist)
    except Exception:
        return 400.0


def load_reference_orb(folder):
    """Load weed reference descriptors using ORB."""
    orb = cv2.ORB_create(nfeatures=ORB_FEATURES)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    refs = []
    for f in os.listdir(folder):
        if not f.lower().endswith((".jpg", ".png", ".jpeg")):
            continue
        img = cv2.imread(os.path.join(folder, f))
        if img is None:
            continue
        img = cv2.resize(img, (200, 200))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kp, des = orb.detectAndCompute(gray, None)
        refs.append((f, des))
    print(f"✅ Loaded {len(refs)} weed references from '{folder}'.")
    return refs, orb, bf


# ============================================================
# MAIN DETECTION LOOP
# ============================================================

def main():
    # ---- Setup ----
    if not os.path.isdir(REF_FOLDER):
        print(f"Error: folder '{REF_FOLDER}' missing.")
        sys.exit(1)

    pwm_base, pwm_arm = setup_gpio()
    refs, orb, bf = load_reference_orb(REF_FOLDER)

    # Camera
    if Picamera2 is None:
        print("❌ Picamera2 not available.")
        return
    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT)}))
    cam.start()
    time.sleep(0.5)

    print("\n🌿 Weed detection started. Press 'q' to exit.\n")
    frame_skip = 0

    try:
        while True:
            frame = cam.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            frame_skip += 1
            if frame_skip % 3 != 0:  # Skip every 2 frames for speed
                cv2.imshow("Weed Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
            mask = cv2.medianBlur(mask, 5)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                roi = frame[y:y+h, x:x+w]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                kp2, des2 = orb.detectAndCompute(gray, None)
                if des2 is None:
                    continue

                best_match = 0
                for name, des1 in refs:
                    matches = bf.match(des1, des2)
                    good = [m for m in matches if m.distance < MATCH_DIST_THRESH]
                    if len(good) > best_match:
                        best_match = len(good)

                # Strong enough match → likely weed
                if best_match >= GOOD_MATCH_COUNT:
                    cx, cy = x + w//2, y + h//2
                    row = int(cy / (FRAME_HEIGHT / GRID_SIZE))
                    col = int(cx / (FRAME_WIDTH / GRID_SIZE))
                    depth = read_lidar()

                    print(f"🌱 Weed detected! Grid cell ({row},{col}), matches={best_match}")
                    move_to_grid(pwm_base, pwm_arm, row, col, depth)
                    activate_cutter()

                    # Draw bounding box
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
                    cv2.putText(frame, "Weed", (x, y-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            # Grid overlay for visual debugging
            for i in range(1, GRID_SIZE):
                cv2.line(frame, (0, i*FRAME_HEIGHT//GRID_SIZE),
                         (FRAME_WIDTH, i*FRAME_HEIGHT//GRID_SIZE), (255,0,0), 1)
                cv2.line(frame, (i*FRAME_WIDTH//GRID_SIZE, 0),
                         (i*FRAME_WIDTH//GRID_SIZE, FRAME_HEIGHT), (255,0,0), 1)

            cv2.imshow("Weed Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user.")

    finally:
        print("Cleaning up...")
        if GPIO:
            GPIO.output(RELAY_PIN, GPIO.LOW)
            GPIO.cleanup()
        cam.close()
        cv2.destroyAllWindows()
        print("✅ Shutdown complete.")


if __name__ == "__main__":
    main()
