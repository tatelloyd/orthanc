# config/settings.py
import os

# Camera settings
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

# Detection settings
CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.4
YOLO_MODEL = 'yolov8n.pt'  # nano model for Pi performance

# Servo settings (GPIO pins)
PAN_SERVO_PIN = 18
TILT_SERVO_PIN = 19

# IMU settings
IMU_I2C_ADDRESS = 0x68

# LED settings
LED_PIN = 21
LED_COUNT = 12

# Tracking settings
MAX_TRACKS = 10
TRACK_TIMEOUT = 30  # frames

# Web interface
WEB_HOST = '0.0.0.0'
WEB_PORT = 5000
DEBUG = True
