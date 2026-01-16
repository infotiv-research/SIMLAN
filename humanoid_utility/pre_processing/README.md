## Dataset preprocessing

- `media_to_pose_landmark.py` : To convert media file to pose landmarks using mediapipe.
- `dataframe_json_bridge.py` : to load poses and motions into a tabulated dataframe.

Models MUST use `dataframe_json_bridge.py` standard interface to load their data.
This tools also has a cli interface to see the intermediate results that can be accessed by `./control.sh convert2csv ...` .

Pose and motion data are first collected from JSON files and combined into a single CSV file. These processed CSV files are ready to be fed into a model of your choice, AutoGluon or PyTorch. See below to find more information about the dataset or more about the [models](#models).

> The dataset, inputs, and outputs are in `humanoid_utility/DATASET/` directory.

The resulting files are stored as parallel `.json` files in multi-camera folders `camera_*/pose_data`, `camera_*/motion_data`, `camera_*/pose_images`.
To avoid creation of `pose_images` that are only used as the ground truth and debugging purposes (especially if you are building your training data), comment out the call to `self.save_pose_image()` in `camera_viewer.py`.

Finally, to merge them all into a tabular `.csv` file, run the following command. The argument `dataset_dir` is the directory containing the JSON files and must be located as a subdirectory of `DATASET/`. Additionally, in `config.sh` you need to set the variable `dataset_cameras` to the cameras you want to take into consideration. i.e., if you use all cameras, the value is "500 501 502 503". The resulting CSV file will then be saved as `dataset_dir/flattened_data.csv`.

By running the command below results in the following csv files being created:

```
./control.sh convert2csv TRAIN
./control.sh convert2csv EVAL
```

Results in merging:

- `DATASET/TRAIN/**` to `DATASET/TRAIN/flattened_data.csv`
- `DATASET/EVAL/**` to `DATASET/EVAL/flattened_data.csv`

> Note: The TRAIN/\*\* refers to the contents of folders `motion_data/` and `camera_5XX/pose_data`.

The resulting CSV file will contain these columns:

```
cam500_0_x,cam501_0_x,cam502_0_x,cam503_0_x,...,cam503_32_z, JOINTS...
```

When the data is transformed into numpy/tabular format, each row in the CSV file represents **multiple pose** samples **from each camera** and its corresponding robot motion. The number of columns depends on the number of cameras, and it's possible to select what camera inputs you want to use; i.e., just one camera input or up to four camera inputs.

- The first columns are the 3D coordinates (x, y, z) for each of the 33 MediaPipe pose landmarks and **for each camera**, named like `cam500_0_x, cam501_0_x, cam502_0_x, cam503_0_x, cam500_0_y, cam501_0_y, cam502_0_y, cam503_0_y, cam500_0_z, cam501_0_z, cam502_0_z, cam503_0_z...`.
- The remaining columns are the robot joint positions (motion targets) for that sample, with names like jRightShoulder_rotx, jLeftElbow_roty, etc.

## Loading Dataset

For training and prediction, we have a helper library, `dataframe_json_bridge.py`, that loads the data into a pandas dataframe with the column size of  `num_pose_landmarks * num_cameras * 3 + num_humanoid_joints`

### Media to pose

```
python3 $humanoid_utility_dir/pre_processing/media_to_pose_landmark.py \
        --mode $2 \ # image or video \
        --data_dir $3 \ # input dataset directory name \
        --camera_ids "$dataset_cameras" \
        --input_dir $humanoid_input_dir \
        --output_dir $humanoid_output_dir
```
