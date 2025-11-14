import os
import glob
import json
import numpy as np
import pandas as pd
import sys

sys.path.append('.')
import humanoid_utility.humanoid_config as humanoid_config


# np_motions = np.empty((0, humanoid_config.NUM_JOINTS))
# np_poses = np.empty((0, humanoid_config.NUM_MARKERS * 3))
CAMERA_IDS = [500, 501, 502, 503]

def collect_motion_id(motion_dir):
    ids = []
    for filename in os.listdir(motion_dir):
        if filename.endswith('_motion.json'):
            ids.append(filename.replace('_motion.json', ''))
    ids.sort()
    return ids

# def load_json_data(pose_dir, motion_dir):
#     '''
#     Loads .json data from pose and motion directories
#     Ensures the motion ID in the filename matches between pose and motion
#     Flattens the pose data to single dict from a list of dicts
#     Returns both pose and motion data as numpy arrays
#     '''
#     all_np_poses = np.empty((0, len(humanoid_config.pose_names)))	
#     all_np_motions = np.empty((0, len(humanoid_config.movable_joint_names)))	

#     # Look for .json files
#     for filename in os.listdir(pose_dir):
#         # Check if it's a file (not a directory)
#         if os.path.isfile(os.path.join(pose_dir, filename)) :
#             id = filename.split("_")[0]
#             # print(id)
#             try:
#                 posefile=os.path.join(pose_dir  ,id + '_pose.json')
#                 motionfile=os.path.join(motion_dir,id + '_motion.json')

#                 np_pose_values,np_pose_keys = load_pose_from_json(posefile)
#                 np_motion_values, np_motion_keys= load_motion_from_json(motionfile)
#             except:
#                 print("WARNING, MISSING FILE for ", id)
#             all_np_poses   = np.append(all_np_poses, [np_pose_values], axis=0)
#             all_np_motions = np.append(all_np_motions, [np_motion_values], axis=0)

#     print(np_poses.shape)
#     print(np_motions.shape)

#     df_all_poses   = pd.DataFrame(all_np_poses, columns=np_pose_keys)
#     df_all_motions = pd.DataFrame(all_np_motions, columns=np_motion_keys)

#     #print(df_all_poses)
#     #print(df_all_motions)
#     return (df_all_poses, df_all_motions)


def load_json_data(pose_dir, motion_dir):
    '''
    Loads .json data from pose and motion directories
    Ensures the motion ID in the filename matches between pose and motion
    Flattens the pose data to single dict from a list of dicts
    Returns both pose and motion data as numpy arrays
    '''
    train_root = os.path.dirname(os.path.dirname(pose_dir))
    motion_ids = collect_motion_id(motion_dir)

    pose_rows = []
    motion_rows = []
    row_camera_ids = []
    motion_id_list = []
    np_pose_keys = None
    np_motion_keys = None

    for motion_id in motion_ids:
        motionfile = os.path.join(motion_dir, f"{motion_id}_motion.json")
        try:
            np_motion_values, np_motion_keys = load_motion_from_json(motionfile)
        except Exception as e:
            print(f"WARNING, MISSING MOTION FILE for {motion_id}: {e}")
            continue

        for camera_id in CAMERA_IDS:
            posefile = os.path.join(train_root, f"camera_{camera_id}", "pose_data", f"{motion_id}_pose.json")
            if not os.path.isfile(posefile):
                print(f"WARNING, MISSING POSE FILE for {motion_id} at camera {camera_id}")
                continue
            try:
                np_pose_values, np_pose_keys = load_pose_from_json(posefile)
            except Exception as e:
                print(f"WARNING, ERROR LOADING POSE FILE for {motion_id} at camera {camera_id}: {e}")
                continue

            pose_rows.append(np_pose_values)
            motion_rows.append(np_motion_values)
            row_camera_ids.append(camera_id)
            motion_id_list.append(motion_id)

    if len(pose_rows) == 0 or len(motion_rows) == 0:
        print("No valid pose or motion data found.")
        return pd.DataFrame(), pd.DataFrame()

    df_all_poses = pd.DataFrame(pose_rows, columns=np_pose_keys)
    df_all_motions = pd.DataFrame(motion_rows, columns=np_motion_keys)
    df_all_motions.insert(0, 'motion_id', motion_id_list)
    df_all_poses.insert(1, 'camera_id', row_camera_ids)

    print(df_all_poses.shape)
    print(df_all_motions.shape)

    return (df_all_poses, df_all_motions)


def load_pose_from_json(filename):
    pdata = open(filename, 'r')
    posedata = json.load(pdata)
    fl_posedata = flatten_pose(posedata)
    return np.array(list(fl_posedata.values())), list(fl_posedata.keys())


def load_motion_from_json(filename):
    mdata = open(filename, 'r')
    motiondata = json.load(mdata)
    return np.array(list(motiondata.values())), list(motiondata.keys())


def flatten_pose(list_of_dicts):
    flattened_dict = {}
    for idx, d in enumerate(list_of_dicts):
        for key, value in d.items():
            if "index" not in key and "visibility" not in key:
                new_key = f"{idx}_{key}"  # Combine index and key
                flattened_dict[new_key] = value
    return flattened_dict


if __name__ == "__main__":
    os.makedirs("humanoid_utility/DATASET_PREPROCESSED", exist_ok=True)

    pose_dir = sys.argv[1]
    motion_dir = sys.argv[2]
    csv_filename = sys.argv[3]
    print("Loading data once at the beginning...")
    pose_dataset, motion_dataset = load_json_data(pose_dir, motion_dir)
    #print(pose_dataset, motion_dataset )
    pose_df   = pd.DataFrame(pose_dataset)
    motion_df = pd.DataFrame(motion_dataset)
    result = pd.concat([pose_df, motion_df], axis=1).reindex(pose_df.index)
    print(result)
    print("dumping to csv file:", csv_filename)
    cols = ['motion_id', 'camera_id'] + [c for c in result.columns if c not in ('motion_id', 'camera_id')]
    result = result[cols].sort_values(by='motion_id')

    result.to_csv(csv_filename, index=False)
