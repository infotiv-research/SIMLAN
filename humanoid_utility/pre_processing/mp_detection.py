import os
from pathlib import Path
import json
import numpy as np
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import argparse
import os
import shutil
import sys

os.environ['MEDIAPIPE_MODEL_PATH'] = str(Path.home() / '.mediapipe/models')
os.makedirs(os.environ['MEDIAPIPE_MODEL_PATH'], exist_ok=True)


def process_single_image(input_image, output_dir):
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    
    pose = mp_pose.Pose(
        static_image_mode=False,  
        model_complexity=2,
        min_detection_confidence=0.01,
        min_tracking_confidence=0.01
    ) 

    frame = cv2.imread(input_image)
    if frame is None:
        print(f"Error: Could not read image {input_image}")
        return
    
    results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    
    if results.pose_landmarks:
        display_image = frame.copy()
        mp_drawing.draw_landmarks(
            display_image,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS
        )
        
        cv2.imwrite(f"{input_image}_display.jpg", display_image)
        
        save_pose_data(results.pose_landmarks, output_dir + '/pose_data/', '000001')
        print(f"Processed single image: {input_image}")
    else:
        print("No pose detected in the image")


def process_video(input_video, output_dir):
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    
    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=2,
        min_detection_confidence=0.01,
        min_tracking_confidence=0.01
    )

    cap = cv2.VideoCapture(input_video)
    frame_count = 0
    frequency = 50
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print('finished')
            break
        frame_count += 1

        if frame_count % frequency != 0:
            continue
        
        results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if results.pose_landmarks:
            display_image = frame.copy()
            mp_drawing.draw_landmarks(
                display_image,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS
            )

            frame_id = f"{frame_count:07}"
            display_filename = f"{frame_id}_display.jpg"
            display_path = os.path.join(output_dir, 'pose_images', display_filename)
            cv2.imwrite(display_path, display_image)

            original_filename = f"{frame_id}_original.png"
            original_path = os.path.join(output_dir, 'pose_images', original_filename)
            cv2.imwrite(original_path, frame)

            save_pose_data(results.pose_landmarks, output_dir + '/pose_data/', frame_id)


def save_pose_data(pose_landmarks, pose_dir, frame_id):
    landmarks_data = []
    for idx, landmark in enumerate(pose_landmarks.landmark):
        landmarks_data.append({
            'index': idx,
            'x': landmark.x,
            'y': landmark.y,
            'z': landmark.z,
            'visibility': landmark.visibility
        })
    
    json_path = os.path.join(pose_dir , f"{frame_id}_pose.json")
    
    with open(json_path, 'w') as f:
        json.dump(landmarks_data, f, indent=2)
    print(f'Saved pose data to {json_path}')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process files from input to output directory.")
    
    parser.add_argument(
        '--action', 
        choices=['image', 'video'], 
        required=True, 
        help='Action to perform on files: image or video'
    )

    parser.add_argument('input_file', help='input file')
    parser.add_argument('output_dir', help='output directory')

    args = parser.parse_args()
    if args.action  == 'image':
        process_single_image(args.input_file, args.output_dir)
    else:
        process_video(args.input_file, args.output_dir)

