## Preprocessing

![pose_landmarks_index.png](resources/pose_landmarks_index.png)

https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker

Three steps of normalization (feature engineering) of MediaPipe landmarks:

- Position normalization
- Scale normalization
- Rotation normalization

![normalization.png](resources/Normalisation.drawio.png)

## Dataset preprocessing

The diagram below shows the overall workflow of this project. Pose and motion data are first collected from JSON files and combined into a single CSV file. These processed CSV files are ready to be fed into a model of your choice, AutoGluon or PyTorch. See below to find more information about the dataset or more about the [models](#models).

> The dataset, inputs, and outputs are in `humanoid_utility/DATASET/` directory.

The resulting files are stored as parallel `.json` files in multi-camera folders `camera_*/pose_data`, `camera_*/motion_data`, `camera_*/pose_images`.
To avoid creation of `pose_images` that are only used as the ground truth and debugging purposes (especially if you are building your training data), comment out the call to `self.save_pose_image()` in `camera_viewer.py`.

Finally, to merge them all into a tabular `.csv` file, run the following command. The argument $dataset_dir is the directory containing the JSON files and must be located as a subdirectory of DATASET/. Additionally, in `config.sh` you need to set the variable `dataset_cameras` to the cameras you want to take into consideration. i.e., if you use all cameras, the value is "500 501 502 503". The resulting CSV file will then be saved as `dataset_dir/flattened_data.csv`.

```
./control.sh convert2csv $dataset_dir
```

Example by running the command below results in the following csv files being created:

```
./control.sh convert2csv TRAIN
./control.sh convert2csv EVAL
```

Results in merging:

- `DATASET/TRAIN/**` to `DATASET/TRAIN/flattened_data.csv`
- `DATASET/EVAL/**` to `DATASET/EVAL/flattened_data.csv`

> Note: The TRAIN/\*\* refers to the folders `motion_data/` and `camera_5XX/pose_data`.

The resulting CSV file will contain these columns:

```
cam500_0_x,cam501_0_x,cam502_0_x,cam503_0_x,...,cam503_32_z, JOINTS...
```

You can replay each motion data file separately: `./control.sh replay_motion DATASET/EVAL/motion_data/AAAAAAA_motion.json`

### Current Performance Metrics

The upgraded multi-camera system demonstrates the following evaluation results:

- **Mean Squared Error (MSE)**: 0.206
- **Root Mean Squared Error (RMSE)**: 0.454

#### Optuna

Optuna is a hyperparameter optimization framework available as a Python library. It acts as a wrapper around an existing training framework where it uses the same setup of dataset, train_loop and hyperparameters, so it has the same context as when you would run it yourself. Optuna starts by defining an objective function. The goal of the objective function is either to minimize or maximize a value. You specify this value yourself; for this setup we have typically used the validation MSE as the objective to minimize. We also need to define the scope of values for the hyperparameters. We define for every hyperparameter a range of values which Optuna is allowed to select from, e.g., BATCHSIZE being in range(4,128). When we have done this for every param we want to find the optimal value for, we start Optuna by running a "study". Optuna runs N number of training sessions, each independent, and uses internal algorithms to find the best set of values for the list of hyperparameters. In the end we get the training session that scored the best and can use its hyperparameters.

Note: We only use this for PyTorch since AutoGluon has its own hyperparameter optimization. This is why we only see studies inside `pytorch/optuna_studies/`.
