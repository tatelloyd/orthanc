#!/usr/bin/env python3
"""
Headless Camera FOV Calibration Tool
Calculates and verifies camera field of view without display.
Saves annotated images for remote verification.

Usage:
    python calibrate_fov_headless.py --method [checkerboard|manual|calculate]
"""

import cv2
import numpy as np
import json
import argparse
import os
from datetime import datetime
from pathlib import Path


class FOVCalibrator:
    def __init__(self, output_dir="fov_calibration_output"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.config_path = Path("config/turret_config.json")
        
    def capture_image(self, camera_id=0, resolution=(640, 480)):
        """Capture a single image from camera"""
        print(f"📷 Opening camera {camera_id}...")
        cap = cv2.VideoCapture(camera_id)
        
        if not cap.isOpened():
            print(f"❌ Could not open camera {camera_id}")
            return None
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        
        # Let camera warm up
        for _ in range(5):
            cap.read()
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            print("❌ Failed to capture frame")
            return None
        
        print(f"✅ Captured {frame.shape[1]}x{frame.shape[0]} image")
        return frame
    
    def save_annotated_image(self, image, filename, annotations=None):
        """Save image with annotations for remote verification"""
        output_path = self.output_dir / filename
        
        if annotations:
            annotated = image.copy()
            h, w = annotated.shape[:2]
            
            # Draw center crosshair
            cv2.line(annotated, (w//2, 0), (w//2, h), (0, 255, 0), 2)
            cv2.line(annotated, (0, h//2), (w, h//2), (0, 255, 0), 2)
            
            # Add text annotations
            y_pos = 30
            for text in annotations:
                cv2.putText(annotated, text, (10, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y_pos += 30
            
            cv2.imwrite(str(output_path), annotated)
        else:
            cv2.imwrite(str(output_path), image)
        
        print(f"💾 Saved: {output_path}")
        return output_path
    
    def calibrate_checkerboard(self, square_size_cm, distance_cm, 
                               pattern_size=(7, 7), camera_id=0):
        """
        Calibrate FOV using checkerboard pattern
        
        Args:
            square_size_cm: Size of one checkerboard square in cm
            distance_cm: Distance from camera to checkerboard in cm
            pattern_size: (width, height) number of inner corners
            camera_id: Camera device ID
        """
        print("\n" + "="*70)
        print("📐 Checkerboard FOV Calibration (Headless Mode)")
        print("="*70)
        print(f"Parameters:")
        print(f"  - Square size: {square_size_cm} cm")
        print(f"  - Distance: {distance_cm} cm")
        print(f"  - Pattern: {pattern_size[0]}x{pattern_size[1]} inner corners")
        print()
        
        # Capture multiple frames and pick best one
        print("📷 Capturing frames...")
        best_frame = None
        best_corners = None
        
        cap = cv2.VideoCapture(camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        for i in range(10):
            ret, frame = cap.read()
            if not ret:
                continue
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret_corners, corners = cv2.findChessboardCorners(
                gray, pattern_size, None
            )
            
            if ret_corners:
                # Refine corner positions
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                best_frame = frame
                best_corners = corners_refined
                print(f"  Frame {i+1}: ✅ Checkerboard detected")
                break
            else:
                print(f"  Frame {i+1}: ❌ No checkerboard")
        
        cap.release()
        
        if best_corners is None:
            print("\n❌ Could not detect checkerboard in any frame")
            print("Tips:")
            print("  - Ensure checkerboard is well-lit and in focus")
            print("  - Center the checkerboard in the camera view")
            print("  - Make sure pattern size matches your checkerboard")
            return None, None
        
        # Draw detected corners
        display_frame = best_frame.copy()
        cv2.drawChessboardCorners(display_frame, pattern_size, best_corners, True)
        
        # Calculate FOV
        corners_reshaped = best_corners.reshape(-1, 2)
        left_corner = corners_reshaped[np.argmin(corners_reshaped[:, 0])]
        right_corner = corners_reshaped[np.argmax(corners_reshaped[:, 0])]
        top_corner = corners_reshaped[np.argmin(corners_reshaped[:, 1])]
        bottom_corner = corners_reshaped[np.argmax(corners_reshaped[:, 1])]
        
        # Real dimensions of the visible checkerboard
        real_width_cm = square_size_cm * (pattern_size[0] - 1)
        real_height_cm = square_size_cm * (pattern_size[1] - 1)
        
        # Calculate FOV using trigonometry: FOV = 2 * arctan(size / (2 * distance))
        fov_h_rad = 2 * np.arctan(real_width_cm / (2 * distance_cm))
        fov_v_rad = 2 * np.arctan(real_height_cm / (2 * distance_cm))
        
        fov_h_deg = np.degrees(fov_h_rad)
        fov_v_deg = np.degrees(fov_v_rad)
        
        # Save annotated image
        annotations = [
            f"Checkerboard: {pattern_size[0]}x{pattern_size[1]}",
            f"Distance: {distance_cm} cm",
            f"FOV: {fov_h_deg:.2f}° x {fov_v_deg:.2f}°"
        ]
        self.save_annotated_image(display_frame, "fov_checkerboard_result.jpg", annotations)
        
        return fov_h_deg, fov_v_deg
    
    def calibrate_manual(self, object_width_cm, distance_cm, camera_id=0):
        """
        Calculate FOV from known object dimensions
        Note: This is a theoretical calculation - for verification, 
        measure the object in the saved image
        
        Args:
            object_width_cm: Width of a known object in cm
            distance_cm: Distance from camera to object in cm
            camera_id: Camera device ID
        """
        print("\n" + "="*70)
        print("📏 Manual FOV Calculation")
        print("="*70)
        print(f"Parameters:")
        print(f"  - Object width: {object_width_cm} cm")
        print(f"  - Distance: {distance_cm} cm")
        print()
        
        frame = self.capture_image(camera_id)
        if frame is None:
            return None, None
        
        h, w = frame.shape[:2]
        aspect_ratio = h / w
        
        # This assumes the object fills the frame width - adjust as needed
        # For accurate measurement, you'd need to mark the object edges
        # Here we provide a theoretical calculation
        
        print("⚠️  This is a theoretical calculation.")
        print("    For accurate measurement, verify with the saved image.")
        print()
        
        # Calculate FOV if object fills frame width
        fov_h_rad = 2 * np.arctan(object_width_cm / (2 * distance_cm))
        fov_h_deg = np.degrees(fov_h_rad)
        fov_v_deg = fov_h_deg * aspect_ratio
        
        # Save reference image with measurement guides
        display_frame = frame.copy()
        
        # Draw measurement guides
        cv2.line(display_frame, (w//2, 0), (w//2, h), (0, 255, 0), 2)
        cv2.line(display_frame, (0, h//2), (w, h//2), (0, 255, 0), 2)
        
        # Draw quarter markers
        for i in range(1, 4):
            x = w * i // 4
            cv2.line(display_frame, (x, h//2-20), (x, h//2+20), (255, 255, 0), 2)
        
        annotations = [
            f"Reference: {object_width_cm} cm at {distance_cm} cm",
            f"Calculated FOV: {fov_h_deg:.2f}° x {fov_v_deg:.2f}°",
            "Verify object edges align with frame edges"
        ]
        self.save_annotated_image(display_frame, "fov_manual_reference.jpg", annotations)
        
        return fov_h_deg, fov_v_deg
    
    def calculate_from_specs(self, sensor_width_mm, focal_length_mm, resolution=(640, 480)):
        """
        Calculate FOV from camera specifications
        
        Args:
            sensor_width_mm: Camera sensor width in mm (e.g., 3.68 for Pi Camera V2)
            focal_length_mm: Lens focal length in mm (e.g., 3.04 for Pi Camera V2)
            resolution: (width, height) in pixels
        """
        print("\n" + "="*70)
        print("🔬 FOV Calculation from Camera Specs")
        print("="*70)
        print(f"Parameters:")
        print(f"  - Sensor width: {sensor_width_mm} mm")
        print(f"  - Focal length: {focal_length_mm} mm")
        print(f"  - Resolution: {resolution[0]}x{resolution[1]}")
        print()
        
        # Calculate horizontal FOV
        fov_h_rad = 2 * np.arctan(sensor_width_mm / (2 * focal_length_mm))
        fov_h_deg = np.degrees(fov_h_rad)
        
        # Calculate vertical FOV using aspect ratio
        aspect_ratio = resolution[1] / resolution[0]
        sensor_height_mm = sensor_width_mm * aspect_ratio
        fov_v_rad = 2 * np.arctan(sensor_height_mm / (2 * focal_length_mm))
        fov_v_deg = np.degrees(fov_v_rad)
        
        # Capture and save reference image
        frame = self.capture_image(0, resolution)
        if frame is not None:
            annotations = [
                f"Sensor: {sensor_width_mm}x{sensor_height_mm:.2f} mm",
                f"Focal: {focal_length_mm} mm",
                f"FOV: {fov_h_deg:.2f}° x {fov_v_deg:.2f}°"
            ]
            self.save_annotated_image(frame, "fov_calculated.jpg", annotations)
        
        return fov_h_deg, fov_v_deg
    
    def verify_fov(self, camera_id=0):
        """
        Capture test image with current FOV settings overlaid
        """
        print("\n" + "="*70)
        print("👁️  FOV Verification")
        print("="*70)
        
        # Load current config
        try:
            with open(self.config_path, 'r') as f:
                config = json.load(f)
                current_fov_h = config['camera']['fov_horizontal']
                current_fov_v = config['camera']['fov_vertical']
            print(f"Current config FOV: {current_fov_h}° x {current_fov_v}°")
        except Exception as e:
            print(f"⚠️  Could not load config: {e}")
            return
        
        frame = self.capture_image(camera_id)
        if frame is None:
            return
        
        h, w = frame.shape[:2]
        display_frame = frame.copy()
        
        # Draw center and angle markers
        cv2.line(display_frame, (w//2, 0), (w//2, h), (0, 255, 0), 2)
        cv2.line(display_frame, (0, h//2), (w, h//2), (0, 255, 0), 2)
        
        # Draw angle grid lines
        for angle_frac in [0.25, 0.5, 0.75]:
            x_pos = int(w * angle_frac)
            angle_deg = (angle_frac - 0.5) * current_fov_h
            cv2.line(display_frame, (x_pos, 0), (x_pos, h), (255, 255, 0), 1)
            cv2.putText(display_frame, f"{angle_deg:.1f}°", (x_pos + 5, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        annotations = [
            f"FOV: {current_fov_h:.1f}° x {current_fov_v:.1f}°",
            "Center = 0°"
        ]
        
        output_path = self.save_annotated_image(display_frame, "fov_verification.jpg", annotations)
        
        print(f"\n✅ Verification image saved")
        print(f"   Check if objects at frame edges appear at ±{current_fov_h/2:.1f}°")
    
    def update_config(self, fov_h, fov_v):
        """Update turret_config.json with new FOV values"""
        try:
            with open(self.config_path, 'r') as f:
                config = json.load(f)
            
            config['camera']['fov_horizontal'] = round(fov_h, 2)
            config['camera']['fov_vertical'] = round(fov_v, 2)
            
            with open(self.config_path, 'w') as f:
                json.dump(config, f, indent=2)
            
            print(f"✅ Updated {self.config_path}")
            print(f"   Horizontal FOV: {fov_h:.2f}°")
            print(f"   Vertical FOV:   {fov_v:.2f}°")
            return True
        except Exception as e:
            print(f"❌ Error updating config: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(
        description="Headless camera FOV calibration tool"
    )
    parser.add_argument(
        '--method',
        choices=['checkerboard', 'manual', 'calculate', 'verify'],
        required=True,
        help='Calibration method'
    )
    parser.add_argument(
        '--square-size',
        type=float,
        help='Checkerboard square size in cm'
    )
    parser.add_argument(
        '--distance',
        type=float,
        help='Distance from camera to target in cm'
    )
    parser.add_argument(
        '--pattern-width',
        type=int,
        default=7,
        help='Checkerboard inner corners width (default: 7)'
    )
    parser.add_argument(
        '--pattern-height',
        type=int,
        default=7,
        help='Checkerboard inner corners height (default: 7)'
    )
    parser.add_argument(
        '--object-width',
        type=float,
        help='Known object width in cm (for manual method)'
    )
    parser.add_argument(
        '--sensor-width',
        type=float,
        help='Camera sensor width in mm (for calculate method)'
    )
    parser.add_argument(
        '--focal-length',
        type=float,
        help='Lens focal length in mm (for calculate method)'
    )
    parser.add_argument(
        '--camera',
        type=int,
        default=0,
        help='Camera device ID (default: 0)'
    )
    parser.add_argument(
        '--update-config',
        action='store_true',
        help='Automatically update config file with results'
    )
    
    args = parser.parse_args()
    
    calibrator = FOVCalibrator()
    
    print("╔════════════════════════════════════════════════════════════╗")
    print("║      HEADLESS CAMERA FOV CALIBRATION TOOL                  ║")
    print("╚════════════════════════════════════════════════════════════╝")
    
    fov_h, fov_v = None, None
    
    if args.method == 'checkerboard':
        if not args.square_size or not args.distance:
            print("❌ Checkerboard method requires --square-size and --distance")
            return 1
        
        fov_h, fov_v = calibrator.calibrate_checkerboard(
            args.square_size,
            args.distance,
            (args.pattern_width, args.pattern_height),
            args.camera
        )
    
    elif args.method == 'manual':
        if not args.object_width or not args.distance:
            print("❌ Manual method requires --object-width and --distance")
            return 1
        
        fov_h, fov_v = calibrator.calibrate_manual(
            args.object_width,
            args.distance,
            args.camera
        )
    
    elif args.method == 'calculate':
        if not args.sensor_width or not args.focal_length:
            print("❌ Calculate method requires --sensor-width and --focal-length")
            return 1
        
        fov_h, fov_v = calibrator.calculate_from_specs(
            args.sensor_width,
            args.focal_length
        )
    
    elif args.method == 'verify':
        calibrator.verify_fov(args.camera)
        return 0
    
    if fov_h is None or fov_v is None:
        print("\n❌ Calibration failed")
        return 1
    
    print("\n" + "="*70)
    print("✅ FOV Calibration Results:")
    print(f"   Horizontal FOV: {fov_h:.2f}°")
    print(f"   Vertical FOV:   {fov_v:.2f}°")
    print("="*70)
    print(f"\n📁 Output saved to: {calibrator.output_dir}/")
    print("   Review the images to verify accuracy")
    
    if args.update_config:
        calibrator.update_config(fov_h, fov_v)
    else:
        print("\n💡 To update config automatically, add --update-config flag")
    
    return 0


if __name__ == "__main__":
    exit(main())