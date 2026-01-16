import json
import os
from pathlib import (
    Path,
)
import sys
import argparse
import subprocess
import cv2
import shutil

sys.path.append(".")

from humanoid_utility import humanoid_config
from humanoid_utility.pose_to_motion.autogluon.framework import AutogluonFramework
from humanoid_utility.pose_to_motion.pytorch.framework import PytorchFramework
from humanoid_utility.pre_processing.media_to_pose_landmark import (
    process_images,
    process_video,
)

from humanoid_utility.pre_processing.dataframe_json_bridge import (
    generate_df_from_json,
    generate_pose_df_from_json_by_ids,
)


def get_directory_filetype(directory: str):
    directory = Path(directory)
    json_directory = Path(directory, "pose_data")

    # Get all files (ignore subdirectories)
    files = [f for f in directory.iterdir() if f.is_file()]
    if json_directory.is_dir():
        files += [f for f in json_directory.iterdir() if f.is_file()]
    if not files:
        return None  # or raise an error

    # Return extension of first file
    return files[0].suffix.lstrip(".")  # without dot


def train(input_dir, model_framework, camera_ids):
    dataset_df = generate_df_from_json(dataset_dir=input_dir, cameras_to_use=camera_ids)
    model_framework.train(input_dir, dataset_df)


def evaluate(input_dir, model_framework, camera_ids):
    # Define dataset
    dataset_df = generate_df_from_json(dataset_dir=input_dir, cameras_to_use=camera_ids)
    model_framework.evaluate(input_dir, dataset_df)


def predict(
    target_dir, model_framework, camera_ids, replay_motions, humanoid_namespace
):
    input_dir = Path("humanoid_utility", "input")
    output_dir = Path("humanoid_utility", "output")
    full_output_target_dir = Path(output_dir, target_dir)

    # Ensure the predicted motion dir exist.
    Path(full_output_target_dir, "predicted_motion").mkdir(parents=True, exist_ok=True)
    input_type = get_directory_filetype(
        str(Path(input_dir, target_dir, f"camera_{camera_ids[0]}"))
    )

    if input_type in ["jpg", "png"]:
        print("Starting to process images...")
        process_images(
            target_dir, camera_ids, input_dir, output_dir, save_output_data=True
        )
    elif input_type in ["mp4"]:
        print("Starting to process videos...")
        process_video(
            target_dir, camera_ids, input_dir, output_dir, save_output_data=True
        )
    elif input_type in ["json"]:
        print("JSON input mode selected, skipping pose extraction...")
        shutil.copytree(
            str(Path(input_dir, target_dir)), full_output_target_dir, dirs_exist_ok=True
        )
    else:
        raise ValueError(f"Unknown input type: {input_type}")

    # all_ids contains all unique ids which we need to predict motion for.
    all_ids = [
        os.path.splitext(f)[0].replace("_pose", "")
        for cam_id in camera_ids
        for f in os.listdir(
            Path(full_output_target_dir, f"camera_{cam_id}", "pose_data")
        )
        if os.path.isfile(
            os.path.join(
                Path(full_output_target_dir, f"camera_{cam_id}", "pose_data"), f
            )
        )
    ]
    all_ids = sorted(list(set(all_ids)))
    print(f"Total {len(all_ids)} unique motion ids found.")

    # We go over all ids and build the input for the model. This using the cameras specified.
    poses_df = generate_pose_df_from_json_by_ids(
        full_output_target_dir, camera_ids, all_ids
    )
    predicted_motions = model_framework.predict(poses_df)
    if len(predicted_motions) != len(poses_df):
        raise IndexError(
            "Number of elements are not the same for predicted motions and input poses"
        )
    # We iterate over all motions we predicted
    for i in poses_df.index:
        motion_id = poses_df["motion_id"][i]  # TODO look over
        with open(
            Path(
                full_output_target_dir, "predicted_motion", f"{motion_id}_motion.json"
            ),
            "w",
        ) as f:
            motion_dict = dict(
                zip(humanoid_config.movable_joint_names, predicted_motions[i])
            )
            print(motion_dict)
            json.dump(motion_dict, f, indent=2)

        if replay_motions:
            # We display the image.
            images_to_show = [
                Path(
                    full_output_target_dir,
                    f"camera_{cam}",
                    "pose_images",
                    f"{motion_id}_original.jpg",
                )
                for cam in camera_ids
            ]
            images_to_show = [path for path in images_to_show if Path.is_file(path)]
            for i, image in enumerate(images_to_show):
                img = cv2.imread(str(image))
                imS = cv2.resize(img, (300, 250))  # Resize image
                # Display the image
                cv2.imshow(f"Image {camera_ids[i]}", imS)

            # Lastly we now run motion_viewer to replay all the predicted motions.
            print("topic:", f"{humanoid_namespace}/execute_motion")

            json_str = json.dumps(motion_dict)
            cmd = [
                "ros2",
                "topic",
                "pub",
                "--once",
                f"{humanoid_namespace}/execute_motion",
                "std_msgs/msg/String",
                f"{{data: '{json_str}'}}",
            ]
            subprocess.run(cmd, check=True)
            cv2.waitKey(15 * 1000)  # this acts as our sleep
            cv2.destroyAllWindows()


def main(args):
    camera_ids = args.camera_ids.split()

    if args.model_type == "pytorch":
        model_framework = PytorchFramework(args.model_instance, camera_ids)
    elif args.model_type == "autogluon":
        model_framework = AutogluonFramework(args.model_instance, camera_ids)
    else:
        raise ValueError(f"Unknown model type: {args.model_type}")

    if args.mode == "train":
        train(args.input_dir, model_framework, camera_ids)
    elif args.mode == "eval":
        evaluate(args.input_dir, model_framework, camera_ids)
    elif args.mode == "predict":
        predict(
            args.input_dir,
            model_framework,
            camera_ids,
            args.replay_motions,
            args.humanoid_namespace,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Model control")

    parser.add_argument(
        "--mode",
        type=str,
        choices=["train", "predict", "eval"],
        required=True,
        help="Mode in which to run the script: train, eval, or predict",
    )
    parser.add_argument(
        "--model_type",
        type=str,
        choices=["pytorch", "autogluon"],
        required=True,
        help="Type of model to use: pytorch or autogluon",
    )
    parser.add_argument(
        "--model_instance",
        type=str,
        help="Model name to either create or load",
        default="",
    )
    parser.add_argument(
        "--input_dir",
        type=str,
        help="Dataset directory path. To be used for train, eval, and predict",
    )
    parser.add_argument(
        "--camera_ids",
        type=str,
        help="List of id as strings, what cameras to use",
    )
    parser.add_argument(
        "--replay_motions",
        action="store_true",
        help="Whether to replay motions",
    )
    parser.add_argument(
        "--humanoid_namespace",
        type=str,
        help="What humanoid robot to apply the predicted motions to.",
    )
    args = parser.parse_args()
    main(args)
