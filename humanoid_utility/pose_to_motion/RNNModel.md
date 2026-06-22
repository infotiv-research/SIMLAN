# Current data structure

What we have now one motion that corresponds to one pose file per camera (def load_json_data(dataset_path, camera_ids)):
dataset_dir/

```
│
├── motion_data/
│   ├── 0001_motion.json
│   ├── 0002_motion.json
│
├── camera_500/
│   └── pose_data/
│       ├── 0001_pose.json
│       ├── 0002_pose.json
│
├── camera_501/
│   └── pose_data/
│       ├── 0001_pose.json
│       ├── 0002_pose.json

```

## Create sequence data

TODO: The data generation code should be changed so that images from several poses (n\<N) during one motion is taken.

What should be updated in humanoid_utility/pre_processing/dataframe_json_bridge.py:

- the input data: should follow the above structure
- load_pose_from_json(): it should parse one frame pose.
- load_pose_sequence(): new function that load all frames of a camera.
- load_json_data(): it should loop over motions
  - what it does now: Loads .json data from pose and motion directories
    Ensures the motion ID in the filename matches between pose and motion
    Flattens the pose data to single dict from a list of dicts
    Returns both pose and motion data as numpy arrays

The final data should be in this shape:
X.shape -> (motions, \[p1_cam, p2_cam,...,pN_cam\])

N: is the length of sequence of poses for one motion

## Recommended RNN models

Three recommended RNN/GRU model architectures in order:

- GRUNet:
  - Lightweight, fast to train
  - Good balance between complexity and performance
  - Recommended for most applications
- LSTMNet:
  - Better long-term dependency learning
  - Slightly slower but more expressive
  - Better for capturing complex motion patterns
- AttentionGRUNet:
  - GRU with attention mechanism
  - Learns which frames in sequence are most important
  - Better interpretability
  - Slightly more computation
