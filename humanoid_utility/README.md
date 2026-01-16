# Humanoid Motion Capture

This project develops a system for translating human pose detection to humanoid robot motion in simulation environments. Using Google MediaPipe for pose landmark detection from camera input, the system maps detected human poses to corresponding joint movements executed by a humanoid robot in the Gazebo simulator. The implementation leverages MoveIt2 for motion planning and control, with a data generation pipeline that creates training pairs of pose landmarks and robot joint configurations.

This project employs a deep neural network to learn the mapping between human pose and humanoid robot motion. The model takes 33 MediaPipe pose landmarks as input and predicts corresponding robot joint positions.

> The work is based on a master's thesis "Human Motion Replay on a Simulated Humanoid Robot Using Pose Estimation" by Tove Casparsson and Siyu Yi, supervised by Hamid Ebadi, June 2025.

## Terminology:

- **Forward Kinematics (FK)**: The process of calculating the position and orientation of a robot's links given the values of its joint parameters (e.g., angles or displacements). In other words, FK answers the question: "Where is the robot's hand if I know all the joint values? (usually have one answer)"
- **Inverse Kinematics (IK)**: The process of determining the joint parameters (e.g., angles or displacements) required to achieve a desired position and orientation of the robot's links. In other words, IK answers the question: "What joint values will place the robot’s hand here? (usually have many answers)"
- **Pose** : 3D pose landmarks (MediaPipe) extracted by MediaPipe from a 2D image of human posture
- **Motion** : A kinematic instruction (joint parameters) sent to the robot (via MoveIt) for execution. While "kinematic instruction" would be a more accurate term, we continue to use "motion" for historical reasons (used in the word motion-capture), even though "motion" often refers to the difference/movement between two postures (e.g., posture2 - posture1).
- **Motion Capture**: Here it means using 2D images to find the motion (kinematic instruction/joint parameters) to instruct a humanoid robot to mimic the human posture.

## Relevant Documents

- [Debug](DEBUG.md) : for debugging purposes and development
- [Preprocessing](pre_processing/README.md) : details about pre processing of datasets
- [motion_planner](/simulation/motion_planner/README.md) - To execute motions in the sim from a motion.json file.

## Dataset generation

To build the synthetic dataset, first ensure that the cameras pointing to the humanoids are active in the `./config.sh`.

```
CAMERA_ENABLED_IDS='500 501 502 503'
dataset_cameras='500 501 502 503'
```

and rebuild the project with these commands:

```
./control.sh build
```

The image below describes how the dataset generation system works.

![Dataset generation overview](resources/dataset_generation.jpg)

To create a humanoid dataset (paired pose data, motion data and reference images) in the `DATASET/TRAIN/` directory:

As of now the dataset generation uses `humanoid_1` hardcoded. TODO: Fix so we can use "replay_motion_namespace".

```bash
./control.sh dataset TRAIN/
./control.sh dataset EVAL/
```

To find the implementation of how the dataset is created i.e. mediapipe, go to [pre_processing/media_to_pose_landmark.py](./pre_processing/media_to_pose_landmark.py).

## Available datasets

We use `20251028-DATASET-TRAINONLY-MULTI.zip` (place it in `DATASET` for training) and `20251028-DATASET-EVAL1000-MULTI.zip` (place it in `input/` directory for prediction) that are available at [SharePoint](https://infotiv-my.sharepoint.com/:f:/r/personal/hamid_ebadi_infotiv_se/Documents/Humanoid?csf=1&web=1&e=M3xnWF) as our common datasets.

## Raw dataset shape

```
DATASET:
── TRAIN
    ├── camera_500
    │   ├── pose_data : 1.json , 2.json
    │   └── pose_images : 1.png , 2.png
    ├── camera_501
    │   ├── pose_data : 1.json , 1.json
    │   └── pose_images : 1.png, 2.png
    └── motion_data: 1.json, 2.json
```

## Cameras

Its important to know how the cameras are located. This is the camera setup for the cameras used for dataset creation, make sure to enable them in `config.sh`:

- 500: Back side
- 501: Right side
- 502: Front side
- 503: Left side

When selecting what cameras to use in "dataset_cameras" in `config.sh` it is important that the order remains the same. To avoid confusion always use the cameras in order: "500 501 502 503" meaning "Back Right Front Left".

## Models

### PyTorch

We have more control over the PyTorch model. It has its own implementation of dataset and model definition, located inside [utils.py](./pose_to_motion/pytorch/utils.py). What is specific about using PyTorch has the possibility to use Optuna is a hyperparameter optimization tool. This tool helps find the best suitable set of hyperparameters given its training data.

- Input Layer:  **33** MediaPipe pose landmarks for each camera (x, y, z coordinates).
- Output Layer: Robot joint position sequences for humanoid motion control, this results in a total of 47 joints to be outputted.

Currently the model hyperparameters (`HyperparameterDict`) in `humanoid_utility/pose_to_motion/pytorch/utils.py` are tuned using Optuna. Future model can use [Optuna](https://optuna.org/#code_examples) for adjusting the  values.

### AutoGluon

We use [Autogluon tabular predictor ](https://auto.gluon.ai/stable/api/autogluon.tabular.TabularPredictor.html). This model is trained as an ensemble (collection) of models where each separate model aims to predict a single joint given the complete input poses. The result from each model is then merged together and becomes a predicted list for each target joint.

## Model Training

To train a model run the command below. The input size depends on the number of cameras you plan on using, meaning that if you only select one camera the input shape will take **1** camera into account. If you select **4** the model will have a **4 times** larger input size. To set it up in the way you want, go to [`config.sh`](config.sh). The variables are:

- `model_instance`: If you want to save a model, specify its name here. Keep blank if you don't want to save.
- `dataset_cameras`: The cameras you want to train on. This can be a list of cameras i.e. "500 501 502". For single training set this to one  ""
- `model_type`: Possible selections: "pytorch", "autogluon"

> Keep in mind that we use a separate virtual environment to install machine learning related pip packages called `mlenv` with a separate [`requirements.txt`](pose_to_motion/MLrequirements.txt). Build the environment first using `./control.sh mlenv`. This has to be done once and `./control.sh` enables the environment automatically

Finally you specify what dataset directory (JSON) you want to use as training data which becomes the second argument, below is an example of running train using the `TRAIN/` dataset.

```
./control.sh mlenv
./control.sh train TRAIN/
```

After a training run is complete, the model is saved inside of `pose_to_motion/{model_type}/output/saved_model_states`. A summary report of the session is also generated and will be saved inside `pose_to_motion/{model_type}/output/train_report.csv`.

## Model Evaluation

```
./control.sh eval EVAL/
```

Running the command above in the terminal runs the evaluation pipeline.

The evaluation will run metrics defined in [metrics.py](./pose_to_motion/metrics.py); currently MSE and MAE are calculated. The results are then saved in a summary report located in `pose_to_motion/{model_type}/outputs/eval_report.csv`.
As the result MSE and MAE values are printed.

### Prediction on synthetic data

The prediction pipeline uses these variables in the `config.sh`:

- `dataset_cameras`: What cameras to take into concern for prediction. Should match the number of cameras the model was trained on.
- `model_instance`: What trained model instance to use.
- `replay_motion_namespace`: What humanoid you want to replay motions on. Default "humanoid_2".

It is possible to predict on three different types of data: images, videos, and raw JSON data.

Run the command below to predict on different types of datasets:

```
./control.sh predict JSON/ # This has the same "EVAL/", "TRAIN/" directories.
./control.sh predict VIDEOS/ # As in input/VIDEOS/
./control.sh predict IMAGES/ # As in input/IMAGES/
```

The input for prediction needs to be structured as the template below.
The camera names are connected with the variable `dataset_cameras` inside [config.sh](/config.sh).

```
input/JSON or VIDEOS or IMAGES
    camera_500/
        file_1.(JSON, PNG, JPG, MP4)
        file_2.(JSON, PNG, JPG, MP4)
    camera_501/
        file_1.(JSON, PNG, JPG, MP4)
        file_2.(JSON, PNG, JPG, MP4)
    camera_502/
        file_1.(JSON, PNG, JPG, MP4)
        file_2.(JSON, PNG, JPG, MP4)
      camera_503/
        file_1.(JSON, PNG, JPG, MP4)
        file_2.(JSON, PNG, JPG, MP4)
```

### Predicting on real data/ your own data

The predict pipeline works on real data as well. It is for example possible to predict this dataset [fit3d\*.tar.gz](https://infotiv-my.sharepoint.com/:f:/r/personal/hamid_ebadi_infotiv_se/Documents/Humanoid?csf=1&web=1&e=M3xnWF) (or alternatively https://fit3d.imar.ro/fit3d.)

If you want to predict on your own videos and images you can follow these steps below. This example will use the gym exercise from 5 angles dataset from [fit3d\*.tar.gz](https://infotiv-my.sharepoint.com/:f:/r/personal/hamid_ebadi_infotiv_se/Documents/Humanoid?csf=1&web=1&e=M3xnWF) or alternatively https://fit3d.imar.ro/fit3d.

1. This example assume you have a trained ML model on 4 cameras. Please follow [humanoid_utility/README.md](humanoid_utility/README.md) for how to train a model.
1. download and extract the [fit3d\*.tar.gz](https://infotiv-my.sharepoint.com/:f:/r/personal/hamid_ebadi_infotiv_se/Documents/Humanoid?csf=1&web=1&e=M3xnWF) dataset.
1. Find a suitable video feed from multiple angles. You can find good examples in the folder "fit3d_train/train/s03/videos/".
   1. In this folder you will find subfolders named "50591643", "58860488", "60457274", "65906101". These represent 4 different camera angles. It is the same when we name our "dataset_cameras" in `config.sh` as "500","501","502","503".
1. Going into any of the subfolders there are many different videos, this example uses "band_pull_apart.mp4"
1. add a new folder inside `input/` with whatever name you want. We will use the name `input/band_pull_apart/`
1. For this guide we will rename the folders "50591643", "58860488", "60457274", "65906101" to "500","501","502","503".
   1. **IMPORTANT**: The angles for this example is not accurate since angles: "50591643" & "58860488" both show back side and "60457274", "65906101" show the front. Thus there become a mismatch from our camera setup where we use back, right, front, left. The ML model will become confused by the angles if not taken into account.
1. Add subfolders inside `input/band_pull_apart/` named after the cameras and add respective video inside of it. This example has 4 videos:
1. `band_pull_apart/camera_500/band_pull_apart.mp4`
1. `band_pull_apart/camera_50X/band_pull_apart.mp4`
1. IMPORTANT: Make sure variable "dataset_cameras" match the names of the folders.
1. Now we can run the following command:

```
./control.sh predict band_pull_apart # based on input/band_pull_apart/
```

The videos will now be processed and the result is found in `output/band_pull_apart/`. Gazebo will also open and execute the motions from the videos.

## Project Structure:

- [`humanoid_config.py`](pose_to_motion/humanoid_config.py): it contains the humanoid configuration like the number of joints and the number of pose landmarks.

- [`humanoid_ml_requirements.txt`](humanoid_ml_requirements.txt): Contains the humanoid machine learning requirements

- [`/DATASET/TRAIN/`](DATASET/TRAIN/): Contains motion, pose and image files generated by `./control.sh dataset TRAIN`

  - `motion_data` : Corresponding random motion (request)
  - `camera/pose_data` : Mediapipe pose (result) for each camera
  - `camera/pose_images`: Mediapipe annotated images and original images for each camera

- [`/DATASET/EVAL/`](DATASET/EVAL/): Same as `TRAIN/`

- [`input/`](input) : This folder has the pose data to be predicted

- [`output/`](output) : This folder saves the predicted motion data and intermediate results

- [`pre_processing/`](pre_processing): contains all pre-processing files, used to convert the JSON data into tabular data.

  - [`media_to_pose_landmark.py`](media_to_pose_landmark.py): Handles pose detection from actual images or videos using MediaPipe
  - [`dataframe_json_bridge.py`](dataframe_json_bridge.py): Contains helper functions to convert json files into dataframe/numpy format and is used by the models. Running its main will generate csv files that contains both pose and motion data from the json files. Inputs the cameras you want to use, the json dir, the csv filename to generate.

- [`pose_to_motion/`](pose_to_motion): Contains the ML training pipeline for pose-to-motion prediction

  - [`model.py`](model.py): The main file containing implementations for training, evaluating, and predicting motions. This file acts as an orchestrator and inputs the data, what model to use, and what operation should be done.

  - [`pytorch/`](pose_to_motion/pytorch): directory containing model implementation for pytorch model. Handles training, evaluation, and predicting motions.

    - [`framework.py`](pose_to_motion/pytorch/model.py): Framework file, contains the implementation for train, evaluation, and prediction.
    - [`utils.py`](pose_to_motion/pytorch/utils.py): Utils file, contains helper methods, and model definition. The `model.py` use this.
    - [`output/`](pose_to_motion/pytorch/output): Contain reports of ran sessions and saved model states. Saved models will be found here
      - [`saved_model_states/`](pose_to_motion/pytorch/output/saved_model_states): Saved autogluon models are stored here
      - [`reports/`](pose_to_motion/pytorch/output/reports): Manually generated reports file containing training/evaluation run information. Good for reproducibility. The reports are generated by `generate_report.py` and the new report row is saved in `train_report.csv` or `eval_report.csv`
      - [`optuna_studies/`](pose_to_motion/pytorch/output/optuna_studies): Specific for pytorch. Optuna studies are stored at this folder. See [optuna section](#optuna) for more information about Optuna.

  - [`autogluon/`](pose_to_motion/autogluon): directory containing model implementation for autogluon model. Handles training, evaluation, and predicting motions. Has similar files as pytorch.

    - [`framework.py`](pose_to_motion/autogluon/model.py)
    - [`utils.py`](pose_to_motion/autogluon/utils.py)
    - [`output/`](pose_to_motion/autogluon/output)
      - [`saved_model_states/`](pose_to_motion/autogluon/output/saved_model_states)
      - [`reports/`](pose_to_motion/autogluon/output/reports)

  - [`generate_report.py`](pose_to_motion/generate_report.py):  Contains methods to generate report files, used after a training or evaluation session has been done for any selected model. These generated reports contain information for reproducing runs and how a model performed against metrics. A new report is generated every time we run train or eval on a model and every report is saved in a single CSV file where every row is a report. The files are named: `train_report.csv` or `eval_report.csv`.

  - [`metrics.py`](pose_to_motion/metrics.py): Contains all functions for calculating metrics for models for any selected model.
