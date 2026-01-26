import os
import json
import numpy as np
import pandas as pd
import sys
import argparse
from pathlib import Path

from os import listdir
from os.path import isfile, join


sys.path.append(".")
import humanoid_utility.humanoid_config as humanoid_config


def flatten_dataframe(default_df):
    # Combined wide CSV for all cameras
    multi_pose_and_motion_df = default_df.pivot(
        index="motion_id", columns="camera_id", values=humanoid_config.pose_names
    )
    multi_pose_and_motion_df.columns = [
        f"cam{cam}_{feat}"
        for feat, cam in multi_pose_and_motion_df.columns.to_flat_index()
    ]
    multi_pose_and_motion_df = multi_pose_and_motion_df.reset_index()

    # Merge with motion data
    motion_df = default_df.drop_duplicates("motion_id")[
        ["motion_id"] + humanoid_config.movable_joint_names
    ]
    result_all = pd.merge(
        multi_pose_and_motion_df, motion_df, on="motion_id", how="left"
    )

    # Sort and drop motion_id for output
    result_all = result_all.sort_values("motion_id").drop("motion_id", axis=1)

    result_all = result_all.fillna(result_all.mean())

    # Return the combined dataframe
    return result_all


def flatten_pose_dataframe(default_df):
    # Combined wide CSV for all cameras
    multi_pose_df = default_df.pivot(
        index="motion_id", columns="camera_id", values=humanoid_config.pose_names
    )
    multi_pose_df.columns = [
        f"cam{cam}_{feat}" for feat, cam in multi_pose_df.columns.to_flat_index()
    ]
    multi_pose_df = multi_pose_df.reset_index()
    cols_to_fill = multi_pose_df.columns[
        1:
    ]  # First col is motion id. So we dont consider it.
    multi_pose_df = multi_pose_df.fillna(multi_pose_df[cols_to_fill].mean())

    # Return the combined dataframe
    return multi_pose_df


def collect_motion_id(motion_dir):
    ids = []
    for filename in os.listdir(motion_dir):
        if filename.endswith("_motion.json"):
            ids.append(filename.replace("_motion.json", ""))
    ids.sort()
    return ids


"""
    Loads .json data from pose and motion directories
    Ensures the motion ID in the filename matches between pose and motion
    Flattens the pose data to single dict from a list of dicts
    Returns both pose and motion data as numpy arrays
"""


def load_json_data(dataset_path, camera_ids):

    motion_dir = dataset_path + "/motion_data"
    motion_ids = collect_motion_id(motion_dir)

    pose_rows = []
    motion_rows = []
    row_camera_ids = []
    motion_id_list = []
    np_pose_keys = None
    np_motion_keys = None

    poses_not_found, motions_not_found = 0, 0
    for motion_id in motion_ids:
        motion_file = os.path.join(motion_dir, f"{motion_id}_motion.json")
        try:
            np_motion_values, np_motion_keys = load_motion_from_json(motion_file)
        except Exception as e:
            # print(f"WARNING, MISSING MOTION FILE for {motion_id}: {e}")
            motions_not_found += 1
            continue

        for camera_id in camera_ids:
            pose_file = os.path.join(
                dataset_path,
                f"camera_{camera_id}",
                "pose_data",
                f"{motion_id}_pose.json",
            )
            if not os.path.isfile(pose_file):
                # print(f"WARNING, MISSING POSE FILE for {motion_id} at camera {camera_id}")
                poses_not_found += 1
                continue
            try:
                np_pose_values, np_pose_keys = load_pose_from_json(pose_file)
            except Exception as e:
                print(
                    f"WARNING, ERROR LOADING POSE FILE for {motion_id} at camera {camera_id}: {e}"
                )
                continue

            pose_rows.append(np_pose_values)
            motion_rows.append(np_motion_values)
            row_camera_ids.append(camera_id)
            motion_id_list.append(motion_id)

    if len(pose_rows) == 0 or len(motion_rows) == 0:
        print("No valid pose or motion data found.")
        return pd.DataFrame(), pd.DataFrame()
    print(
        f"Number of motions found: {len(pose_rows)}, no. missing poses: {poses_not_found}, no. missing motions: {motions_not_found}"
    )

    df_all_poses = pd.DataFrame(pose_rows, columns=np_pose_keys)
    df_all_motions = pd.DataFrame(motion_rows, columns=np_motion_keys)
    df_all_motions.insert(0, "motion_id", motion_id_list)
    df_all_poses.insert(1, "camera_id", row_camera_ids)

    combined_pose_motion_df = pd.concat([df_all_poses, df_all_motions], axis=1).reindex(
        df_all_poses.index
    )
    cols = ["motion_id", "camera_id"] + [
        c
        for c in combined_pose_motion_df.columns
        if c not in ("motion_id", "camera_id")
    ]
    result = combined_pose_motion_df[cols].sort_values(by="motion_id")
    return result


# Inputs a pose filename and outputs a 2D array of (values, keys) i.e ([0.01, 0.02],[x_0, y_0])
def load_pose_from_json(pose_filename):
    # We open the json file
    pose_data_file = open(pose_filename, "r")
    # load json content
    pose_data = json.load(pose_data_file)
    # Flatten the list of dicts into a single dict: Array<Dict> => Dict
    flattened_pose_data = flatten_pose(pose_data)
    return np.array(list(flattened_pose_data.values())), list(
        flattened_pose_data.keys()
    )


# Inputs a motion filename and outputs a 2D array of (values, joint_keys)
def load_motion_from_json(motion_filename):
    motion_data_file = open(motion_filename, "r")
    motion_data = json.load(motion_data_file)
    return np.array(list(motion_data.values())), list(motion_data.keys())


# Inputs a POSE list of dicts and outputs a flattened dicts: Array<Dict> => Dict
def flatten_pose(list_of_dicts):
    flattened_dict = {}
    for idx, d in enumerate(list_of_dicts):
        for key, value in d.items():
            if "index" not in key and "visibility" not in key:
                new_key = f"{idx}_{key}"  # Combine index and key
                flattened_dict[new_key] = value
    return flattened_dict


"""_summary_ Inputs the directory containing the train/eval json data.
    It outputs a flattened dataframe ready for the model to use.
"""


def generate_df_from_json(dataset_dir="TRAIN", cameras_to_use=[500, 501, 502, 503]):

    default_structure_df = load_json_data(dataset_dir, cameras_to_use)
    flattened_df = flatten_dataframe(default_structure_df)
    return flattened_df


# Inputs dataset dir of json files, motion id, and cameras, and returns the poses of that ID and cameras merged into one row.
def generate_pose_df_from_json(dataset_dir, cameras_to_use):

    poses_df = pd.DataFrame(
        columns=["motion_id", "camera_id"] + humanoid_config.pose_names
    )
    for cam_id in cameras_to_use:

        pose_file_dir = os.path.join(
            dataset_dir,
            f"camera_{cam_id}",
            "pose_data",
        )
        pose_files = [
            f for f in listdir(pose_file_dir) if isfile(join(pose_file_dir, f))
        ]
        for filename in pose_files:
            motion_id = filename.split("_")[0]
            new_pose_landmarks, _ = load_pose_from_json(Path(pose_file_dir, filename))
            pose_to_add = [motion_id, cam_id] + new_pose_landmarks.tolist()
            poses_df.loc[len(poses_df)] = pose_to_add

    return poses_df


# Run main if you want to save data
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="convert json, landmarks, motions dictionaries into dataframe/numpy format"
    )

    parser.add_argument("--dataset_dir", help="location of the dataset to use")
    parser.add_argument("--csv_filename", help="filename of the saved csv file")
    parser.add_argument("--camera_ids", help="what cameras you want to use")
    args = parser.parse_args()

    print("Loading data once at the beginning...")
    camera_ids = args.camera_ids.split()

    # Get the final flattened df where each row is one frame ID with each cameras poses & lastly the motion
    flattened_df = generate_df_from_json(
        dataset_dir=args.dataset_dir, cameras_to_use=camera_ids
    )
    flattened_df.to_csv(args.csv_filename, index=False)
