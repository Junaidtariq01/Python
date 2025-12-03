import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
from datetime import datetime
import csv
from smbus2 import SMBus
import VL53L0X  # install with: sudo pip3 install vl53l0x

# =========================
# USER CONFIGURABLE SETTINGS
# =========================
ARM_BASE_PIN = 17
ARM_JOINT_PIN = 27
RELAY_PIN = 22
FPS_DISPLAY_INTERVAL = 1.0
ARM_LENGTH_1 = 10.0   # cm
ARM_LENGTH_2 = 10.0   # cm
GRID_SIZE = 7
CAMERA_INDEX = 0
REFERENCE_IMG_FOLDER = "weed_samples"  # Folder with weed sample images
DEPTH_SAVE_FILE = "grid_depth.csv"

# =========================
# GPIO INITIALIZATION
# =========================
GPIO.setmode(GPIO.BCM)
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.LOW)

GPIO.setup(ARM_BASE_PIN, GPIO.OUT)
GPIO.setup(ARM_JOINT_PIN, GPIO.OUT)
pwm_base = GPIO.PWM(ARM_BASE_PIN, 50)
pwm_arm = GPIO.PWM(ARM_JOINT_PIN, 50)
pwm_base.start(0)
pwm_arm.start(0)

# =========================
# LIDAR SETUP
# =========================
tof = VL53L0X.VL53L0X()
tof.open()
tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

# =========================
# CAMERA INIT
# =========================
from picamera2 import Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(2)

# =========================
# LOAD REFERENCE WEED IMAGES
# =========================
ref_images = []
for file in os.listdir(REFERENCE_IMG_FOLDER):
    if file.endswith((".jpg", ".png", ".jpeg")):
        img = cv2.imread(os.path.join(REFERENCE_IMG_FOLDER, file), cv2.IMREAD_GRAYSCALE)
        ref_images.append(img)

orb = cv2.ORB_create()
ref_keypoints, ref_descriptors = [], []
for ref in ref_images:
    kp, des = orb.detectAndCompute(ref, None)
    ref_keypoints.append(kp)
    ref_descriptors.append(des)

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# =========================
# GRID + DATA VARIABLES
# =========================
grid_depth = np.zeros((GRID_SIZE, GRID_SIZE))
last_save_time = time.time()
save_interval = 10  # seconds

# =========================
# FUNCTIONS
# =========================
def move_servo(pwm, angle):
    duty = 2 + (angle / 18)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.3)

def move_to_grid(row, col, depth):
    # Simple proportional movement — calibrate these constants
    base_angle = np.interp(col, [0, GRID_SIZE-1], [0, 180])
    arm_angle = np.interp(depth, [0, 50], [45, 90])
    move_servo(pwm_base, base_angle)
    move_servo(pwm_arm, arm_angle)

def activate_cutter():
    GPIO.output(RELAY_PIN, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(RELAY_PIN, GPIO.LOW)

def save_grid_data():
    with open(DEPTH_SAVE_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        for row in grid_depth:
            writer.writerow(row)
    print("[INFO] Grid depth data saved at", datetime.now().strftime("%H:%M:%S"))

def detect_weed(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kp2, des2 = orb.detectAndCompute(gray, None)
    max_matches = 0
    for des in ref_descriptors:
        if des is None or des2 is None:
            continue
        matches = bf.match(des, des2)
        if len(matches) > max_matches:
            max_matches = len(matches)
    return max_matches > 20  # adjustable threshold

# =========================
# MAIN LOOP
# =========================
prev_time = time.time()
fps_counter = 0
fps = 0

while True:
    
    frame = picam2.capture_array()

    h, w = frame.shape[:2]
    cell_h, cell_w = h // GRID_SIZE, w // GRID_SIZE
    weed_found = False

    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            y1, y2 = i * cell_h, (i + 1) * cell_h
            x1, x2 = j * cell_w, (j + 1) * cell_w
            cell = frame[y1:y2, x1:x2]

            if detect_weed(cell):
                weed_found = True
                distance = tof.get_distance()
                grid_depth[i][j] = round(distance / 10.0, 2)  # convert mm to cm
                move_to_grid(i, j, grid_depth[i][j])
                activate_cutter()
                color = (0, 0, 255)
            else:
                color = (0, 255, 0)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)

    # FPS Calculation
    fps_counter += 1
    if (time.time() - prev_time) > FPS_DISPLAY_INTERVAL:
        fps = fps_counter / (time.time() - prev_time)
        fps_counter = 0
        prev_time = time.time()

    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.imshow("Weed Detection Grid", frame)

    # Save grid depth periodically
    if time.time() - last_save_time > save_interval:
        save_grid_data()
        last_save_time = time.time()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
tof.stop_ranging()
tof.close()
cap.release()
cv2.destroyAllWindows()
pwm_base.stop()
pwm_arm.stop()
GPIO.cleanup()
