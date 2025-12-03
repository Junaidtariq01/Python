#!/usr/bin/env python3
"""
World-Grid + LiDAR + OpenCV Weed Detection + Servo Arm + Relay (Optimized + Inverse Kinematics)
for Raspberry Pi Zero 2 W + 5MP Pi Camera + VL53L0X

Author: Junii + ChatGPT (2025)
"""

import cv2
import numpy as np
import os, time, math, sys

# ========== Optional Hardware Modules ==========
try:
    from picamera2 import Picamera2
except Exception:
    Picamera2 = None
    print("⚠️ Picamera2 not found, running in simulation mode.")

try:
    import RPi.GPIO as GPIO
except Exception:
    GPIO = None
    print("⚠️ GPIO not available, servo/relay simulated.")

try:
    import vl53l0x
    LIDAR_AVAILABLE = True
except Exception:
    LIDAR_AVAILABLE = False
    print("⚠️ VL53L0X library missing, using mock distances.")

# ============================================================
# USER CONFIGURATION SECTION
# ============================================================

GRID_SIZE = 7
FRAME_WIDTH, FRAME_HEIGHT = 320, 240

# Arm lengths (cm)
ARM1_LENGTH = 10.0
ARM2_LENGTH = 10.0
ARM_TOTAL = ARM1_LENGTH + ARM2_LENGTH

# Grid depth matrix (stores LiDAR distances)
grid_depth = np.zeros((GRID_SIZE, GRID_SIZE))

# HSV mask for detecting green weed areas
HSV_LOWER = np.array([30, 60, 40])
HSV_UPPER = np.array([85, 255, 255])

# Detection & ORB parameters
MIN_CONTOUR_AREA = 250
ORB_FEATURES = 200
GOOD_MATCH_COUNT = 15
MATCH_DIST_THRESH = 50

# Servo GPIO setup
SERVO_BASE = 17
SERVO_ARM = 18
SERVO_FREQ = 50
BASE_RANGE = (30, 150)
ARM_RANGE = (50, 120)

# Relay setup (for cutter motor)
RELAY_PIN = 27
CUT_DURATION = 1.0  # seconds

# Folder with weed reference images
REF_FOLDER = "weeds"

# ============================================================
# INITIALIZATION FUNCTIONS
# ============================================================

def setup_gpio():
    """Setup servos and relay pins."""
    if GPIO is None:
        return None, None
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_BASE, GPIO.OUT)
    GPIO.setup(SERVO_ARM, GPIO.OUT)
    GPIO.setup(RELAY_PIN, GPIO.OUT)
    pwm_base = GPIO.PWM(SERVO_BASE, SERVO_FREQ)
    pwm_arm = GPIO.PWM(SERVO_ARM, SERVO_FREQ)
    pwm_base.start(0)
    pwm_arm.start(0)
    GPIO.output(RELAY_PIN, GPIO.LOW)
    return pwm_base, pwm_arm

def angle_to_duty(angle):
    return 2 + (angle / 18.0)

def set_servo(pwm, angle):
    """Rotate servo to given angle."""
    if pwm is None:
        return
    pwm.ChangeDutyCycle(angle_to_duty(angle))
    time.sleep(0.25)
    pwm.ChangeDutyCycle(0)

def activate_cutter():
    """Turn ON cutter motor via relay for short time."""
    if GPIO is None:
        print("[SIM] Cutter activated.")
        time.sleep(CUT_DURATION)
        return
    GPIO.output(RELAY_PIN, GPIO.HIGH)
    print("🌀 Cutter ON")
    time.sleep(CUT_DURATION)
    GPIO.output(RELAY_PIN, GPIO.LOW)
    print("✂️ Cutter OFF")

def read_lidar():
    """Get distance in mm from VL53L0X."""
    if not LIDAR_AVAILABLE:
        return np.random.uniform(300, 500)  # simulate
    try:
        sensor = vl53l0x.VL53L0X(i2c_bus=1)
        sensor.open()
        sensor.start_ranging(vl53l0x.VL53L0X_BETTER_ACCURACY_MODE)
        d = sensor.get_distance()
        sensor.stop_ranging()
        return float(d)
    except Exception:
        return 400.0

def load_reference_orb(folder):
    """Load reference weed features."""
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
        _, des = orb.detectAndCompute(gray, None)
        refs.append((f, des))
    print(f"✅ Loaded {len(refs)} reference weeds.")
    return refs, orb, bf

# ============================================================
# INVERSE KINEMATICS + GRID CONTROL
# ============================================================

def move_to_grid(pwm_base, pwm_arm, row, col, distance_mm):
    """Calculate servo angles from grid + LiDAR depth."""
    # Convert distance to cm
    distance_cm = distance_mm / 10.0
    grid_depth[row][col] = distance_cm  # store average grid depth

    # Basic inverse kinematics
    x = (col - GRID_SIZE / 2) * 2  # horizontal offset
    y = (row - GRID_SIZE / 2) * 2  # vertical offset (for mapping)
    reach = min(distance_cm, ARM_TOTAL)

    try:
        # Law of cosines for 2-link arm
        cos_theta2 = (reach**2 - ARM1_LENGTH**2 - ARM2_LENGTH**2) / (2 * ARM1_LENGTH * ARM2_LENGTH)
        cos_theta2 = np.clip(cos_theta2, -1, 1)
        theta2 = math.degrees(math.acos(cos_theta2))

        k1 = ARM1_LENGTH + ARM2_LENGTH * math.cos(math.radians(theta2))
        k2 = ARM2_LENGTH * math.sin(math.radians(theta2))
        theta1 = math.degrees(math.atan2(y, x)) - math.degrees(math.atan2(k2, k1))

        base_angle = np.interp(col, [0, GRID_SIZE - 1], BASE_RANGE)
        arm_angle = np.clip(theta2, ARM_RANGE[0], ARM_RANGE[1])

        print(f"→ Grid=({row},{col}) | Dist={distance_cm:.1f}cm | θ1={theta1:.1f}° | θ2={theta2:.1f}°")

        set_servo(pwm_base, base_angle)
        set_servo(pwm_arm, arm_angle)
    except ValueError:
        print("⚠️ Invalid IK solution, skipping cell.")

# ============================================================
# MAIN LOOP
# ============================================================

def main():
    if not os.path.isdir(REF_FOLDER):
        print(f"Error: '{REF_FOLDER}' folder not found.")
        sys.exit(1)

    pwm_base, pwm_arm = setup_gpio()
    refs, orb, bf = load_reference_orb(REF_FOLDER)

    if Picamera2 is None:
        print("❌ Picamera2 missing.")
        return

    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT)}))
    cam.start()
    time.sleep(0.5)

    print(f"\n🌿 OpenCV Weed Mapping | Grid={GRID_SIZE}x{GRID_SIZE} | Arm=({ARM1_LENGTH}+{ARM2_LENGTH})cm | Press 'q' to exit\n")

    frame_counter = 0
    fps_smooth = 0
    last_time = time.time()

    try:
        while True:
            frame = cam.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame_counter += 1

            # FPS calculation (smoothed)
            now = time.time()
            fps = 1.0 / (now - last_time)
            last_time = now
            fps_smooth = (fps_smooth * 0.9) + (fps * 0.1)

            # Weed color detection
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
                _, des2 = orb.detectAndCompute(gray, None)
                if des2 is None:
                    continue

                best_match = 0
                for _, des1 in refs:
                    matches = bf.match(des1, des2)
                    good = [m for m in matches if m.distance < MATCH_DIST_THRESH]
                    best_match = max(best_match, len(good))

                if best_match >= GOOD_MATCH_COUNT:
                    cx, cy = x + w//2, y + h//2
                    row = int(cy / (FRAME_HEIGHT / GRID_SIZE))
                    col = int(cx / (FRAME_WIDTH / GRID_SIZE))
                    depth = read_lidar()
                    move_to_grid(pwm_base, pwm_arm, row, col, depth)
                    activate_cutter()

                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
                    cv2.putText(frame, "Weed", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            # Draw grid lines
            for i in range(1, GRID_SIZE):
                cv2.line(frame, (0, i*FRAME_HEIGHT//GRID_SIZE),
                         (FRAME_WIDTH, i*FRAME_HEIGHT//GRID_SIZE), (255,0,0), 1)
                cv2.line(frame, (i*FRAME_WIDTH//GRID_SIZE, 0),
                         (i*FRAME_WIDTH//GRID_SIZE, FRAME_HEIGHT), (255,0,0), 1)

            # Display FPS
            cv2.putText(frame, f"FPS: {fps_smooth:.1f}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
            cv2.imshow("Weed Detection Grid", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n🛑 Stopped by user.")
    finally:
        if GPIO:
            GPIO.output(RELAY_PIN, GPIO.LOW)
            GPIO.cleanup()
        cam.close()
        cv2.destroyAllWindows()
        print("✅ Shutdown complete.")

if __name__ == "__main__":
    main()
