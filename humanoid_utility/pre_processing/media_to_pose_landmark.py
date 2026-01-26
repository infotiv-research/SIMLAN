import os
from pathlib import Path
import json
import cv2
import mediapipe as mp
import argparse
import os
import pandas as pd
import numpy as np

os.environ["MEDIAPIPE_MODEL_PATH"] = str(Path.home() / ".mediapipe/models")
os.makedirs(os.environ["MEDIAPIPE_MODEL_PATH"], exist_ok=True)
import sys

sys.path.append(".")
from humanoid_utility import humanoid_config


# Inputs an image file and outputs a dictionary of detected landmarks.
# Optionally possible to save a reference image with detected landmarks.
def process_images(data_dir, camera_ids, input_dir, output_dir, save_output_data=True):

    pose_landmarks_df_columns = [
        "motion_id",
        "camera_id",
    ] + humanoid_config.pose_names
    pose_landmarks_df = pd.DataFrame(columns=pose_landmarks_df_columns)

    # Mediapipe setup
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=2,
        min_detection_confidence=0.01,
        min_tracking_confidence=0.01,
    )

    # We look at the first camera dir and store all image filenames as a list which we can iterate over
    first_cam_dir = Path(input_dir, data_dir, f"camera_{camera_ids[0]}")
    all_images = [
        os.path.splitext(f)[0]
        for f in os.listdir(first_cam_dir)
        if os.path.isfile(os.path.join(first_cam_dir, f))
    ]
    all_images = [
        f
        for f in os.listdir(first_cam_dir)
        if os.path.isfile(os.path.join(first_cam_dir, f))
    ]

    # We create the output dirs if we want to save
    if save_output_data:
        Path(output_dir, data_dir).mkdir(parents=True, exist_ok=True)
        for camera_id in camera_ids:
            Path(output_dir, data_dir, f"camera_{camera_id}").mkdir(
                parents=True, exist_ok=True
            )
            Path(output_dir, data_dir, f"camera_{camera_id}/pose_data").mkdir(
                parents=True, exist_ok=True
            )
            Path(output_dir, data_dir, f"camera_{camera_id}/pose_images").mkdir(
                parents=True, exist_ok=True
            )

    # For every image we found we want to apply mediapipe and get the landmarks. This is done for each camera as well.
    for image_id, image_name in enumerate(all_images):
        for camera_id in camera_ids:
            new_pose_landmark_row = []
            input_image_filename = Path(
                input_dir, data_dir, f"camera_{camera_id}", f"{image_name}"
            )
            output_image_filename = Path(
                output_dir, data_dir, f"camera_{camera_id}", "pose_images"
            )
            output_pose_filename = Path(
                output_dir,
                data_dir,
                f"camera_{camera_id}",
                "pose_data",
                f"{image_id:08d}_pose.json",
            )

            if not input_image_filename.is_file():
                print(f"Error: image does not exists: {input_image_filename}")
                continue
            frame = cv2.imread(str(input_image_filename))

            results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            # Saves images to output/ dir
            if results.pose_landmarks:
                display_image = frame.copy()
                mp_drawing.draw_landmarks(
                    display_image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS
                )
                if save_output_data:
                    cv2.imwrite(
                        str(output_image_filename) + f"/{image_id:08d}_display.jpg",
                        display_image,
                    )
                    cv2.imwrite(
                        str(output_image_filename) + f"/{image_id:08d}_original.jpg",
                        frame,
                    )
                    save_pose_data(results.pose_landmarks, output_pose_filename)

                # image id, camera id, pose landmarks.
                # Fill the new row with landmark data
                new_pose_landmark_row += [f"{image_id:08d}", camera_id]
                for landmark in results.pose_landmarks.landmark:
                    new_pose_landmark_row += [landmark.x, landmark.y, landmark.z]

            # No pose found, filling new row with blank data
            else:
                print("No landmarks detected")
                new_pose_landmark_row = [f"{image_id:08d}", camera_id] + [np.nan] * len(
                    humanoid_config.pose_names
                )
            pose_landmarks_df.loc[len(pose_landmarks_df)] = new_pose_landmark_row

    return pose_landmarks_df


# Inputs a video feed and outputs a dictionary of (frame_id, pose_landmarks)
# Optionally you can choose to save reference images with landmarks.
def process_video(data_dir, camera_ids, input_dir, output_dir, save_output_data=True):

    pose_landmarks_df_columns = ["motion_id", "camera_id"] + humanoid_config.pose_names
    pose_landmarks_df = pd.DataFrame(columns=pose_landmarks_df_columns)

    # Mediapipe setup
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=2,
        min_detection_confidence=0.01,
        min_tracking_confidence=0.01,
    )

    # We look at the first camera dir and store all image filenames as a list which we can iterate over
    first_cam_dir = Path(input_dir, data_dir, f"camera_{camera_ids[0]}")
    all_videos = [
        os.path.splitext(f)[0]
        for f in os.listdir(first_cam_dir)
        if os.path.isfile(os.path.join(first_cam_dir, f))
    ]

    # We create the output dirs if we want to save
    if save_output_data:
        Path(output_dir, data_dir).mkdir(parents=True, exist_ok=True)
        for camera_id in camera_ids:
            Path(output_dir, data_dir, f"camera_{camera_id}").mkdir(
                parents=True, exist_ok=True
            )
            Path(output_dir, data_dir, f"camera_{camera_id}/pose_data").mkdir(
                parents=True, exist_ok=True
            )
            Path(output_dir, data_dir, f"camera_{camera_id}/pose_images").mkdir(
                parents=True, exist_ok=True
            )

    # We iterate over all videos for each camera and apply mediapipe on them
    for video_name in all_videos:
        for camera_id in camera_ids:

            input_video_filename = Path(
                input_dir, data_dir, f"camera_{camera_id}", f"{video_name}.mp4"
            )
            output_image_filename = Path(
                output_dir, data_dir, f"camera_{camera_id}", "pose_images"
            )
            output_pose_filename = Path(
                output_dir, data_dir, f"camera_{camera_id}", "pose_data"
            )

            cap = cv2.VideoCapture(str(input_video_filename))
            frame_count = 0
            frequency = 50

            while cap.isOpened():
                new_pose_landmark_row = []
                ret, frame = cap.read()
                if not ret:
                    print("finished")
                    break
                frame_count += 1

                if frame_count % frequency != 0:
                    continue

                results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

                if results.pose_landmarks:
                    display_image = frame.copy()
                    mp_drawing.draw_landmarks(
                        display_image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS
                    )

                    frame_id = f"{frame_count:07}"
                    # Save images to output folder
                    if save_output_data:
                        cv2.imwrite(
                            str(output_image_filename) + f"/{frame_id}_display.jpg",
                            display_image,
                        )
                        cv2.imwrite(
                            str(output_image_filename) + f"/{frame_id}_original.jpg",
                            frame,
                        )
                        # Save landmarks to output folder
                        save_pose_data(
                            results.pose_landmarks,
                            str(output_pose_filename) + f"/{frame_id}_pose.json",
                        )

                    # image id, camera id, pose landmarks
                    # Fill new row with landmark data.
                    new_pose_landmark_row += [f"{frame_id}", camera_id]
                    for landmark in results.pose_landmarks.landmark:
                        new_pose_landmark_row += [landmark.x, landmark.y, landmark.z]
                # No pose found, filling new row with NaN data
                else:
                    print("No landmarks detected")
                    new_pose_landmark_row = [f"{frame_id}", camera_id] + [np.nan] * len(
                        humanoid_config.pose_names
                    )

                pose_landmarks_df.loc[len(pose_landmarks_df)] = new_pose_landmark_row
    return pose_landmarks_df


def save_pose_data(pose_landmarks, pose_filename):
    landmarks_data = []
    for idx, landmark in enumerate(pose_landmarks.landmark):
        landmarks_data.append(
            {
                "index": idx,
                "x": landmark.x,
                "y": landmark.y,
                "z": landmark.z,
                "visibility": landmark.visibility,
            }
        )

    # json_path = os.path.join(pose_dir, f"{frame_id}_pose.json")

    with open(pose_filename, "w") as f:
        json.dump(landmarks_data, f, indent=2)


# Running this runs landmark detection on an input and saves json and display images to chosen output directory.
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Process files from input to output directory."
    )

    parser.add_argument(
        "--action",
        choices=["image", "video"],
        required=True,
        help="Action to perform on files: image or video",
    )

    parser.add_argument("--data_dir", help="name of the dir to predict")
    parser.add_argument("--camera_ids", help="What cameras to take into consideration.")
    parser.add_argument(
        "--input_dir", help="input directory, where to look for data", default="input"
    )
    parser.add_argument(
        "--output_dir", help="output directory, where to save data", default="output"
    )

    args = parser.parse_args()
    camera_ids = args.camera_ids.split()
    if args.action == "image":
        process_images(
            args.data_dir,
            camera_ids,
            args.input_dir,
            args.output_dir,
            save_output_data=True,
        )
    elif args.action == "video":
        process_video(
            args.data_dir,
            camera_ids,
            args.input_dir,
            args.output_dir,
            save_output_data=True,
        )
