"""
Simple configuration for Phase 0: Camera + Detection only
"""

# Camera settings
CAMERA_INDEX = 0  # Change this if your camera is on a different index
CAMERA_WIDTH = 640  # Lower resolution for better Pi performance
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# YOLO Detection settings
MODEL_PATH = 'yolov8n.pt'  # Nano model for best Pi performance
CONFIDENCE_THRESHOLD = 0.5  # Minimum confidence for detections
NMS_THRESHOLD = 0.4  # Non-max suppression threshold

# Target classes to track (from COCO dataset)
TARGET_CLASSES = [
    'person',
    'bicycle', 
    'car',
    'motorcycle',
    'airplane',
    'bus',
    'train',
    'truck',
    'boat'
]

# Display settings
SHOW_FPS = True
SHOW_COORDINATES = True
SAVE_DETECTIONS = False  # Save images with detections

# Performance settings for Raspberry Pi
USE_GPU = False  # Pi doesn't have CUDA
NUM_THREADS = 4  # Pi 4 has 4 cores
SKIP_FRAMES = 0  # Process every frame (increase if too slow)

# Future hardware pins (for reference)
# These aren't used yet in Phase 0
FUTURE_PINS = {
    'PAN_SERVO': 18,
    'TILT_SERVO': 19,
    'LASER': 26,
    'LED_DATA': 21,
    'IMU_SDA': 2,
    'IMU_SCL': 3
}
