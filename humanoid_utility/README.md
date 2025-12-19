# Humanoid Motion Capture

This project develops a system for translating human pose detection to humanoid robot motion in simulation environments. Using Google MediaPipe for pose landmark detection from camera input, the system maps detected human poses to corresponding joint movements executed by a humanoid robot in the Gazebo simulator. The implementation leverages ROS2 Ignition and MoveIt2 for motion planning and control, with a data generation pipeline that creates training pairs of pose landmarks and robot joint configurations. This approach provides a foundation for safety monitoring applications in industrial simulation (SIMLAN), where human pose analysis can be integrated for workplace incident detection. The work is based on a master's thesis "Human Motion Replay on a Simulated Humanoid Robot Using Pose Estimation" by Tove Casparsson and Siyu Yi, supervised by Hamid Ebadi, June 2025.

## Pose to humanoid motion

This project employs a deep neural network to learn the mapping between human pose and humanoid robot motion. The model takes 33 MediaPipe pose landmarks as input and predicts corresponding robot joint positions.

## Terminology:

- **Forward Kinematics (FK)**: The process of calculating the position and orientation of a robot's links given the values of its joint parameters (e.g., angles or displacements). In other words, FK answers the question: "Where is the robot's hand if I know all the joint values? (usually have one answer)"
- **Inverse Kinematics (IK)**: The process of determining the joint parameters (e.g., angles or displacements) required to achieve a desired position and orientation of the robot's links. In other words, IK answers the question: "What joint values will place the robot’s hand here? (usually have many answers)"
- **Pose** : 3D pose landmarks (MediaPipe) extracted by MediaPipe from a 2D image of human posture
- **Motion** : A kinematic instruction (joint parameters) sent to the robot (via MoveIt) for execution. While "kinematic instruction" would be a more accurate term, we continue to use "motion" for historical reasons (used in the word motion-capture), even though "motion" often refers to the difference/movement between two postures (e.g., posture2 - posture1).
- **Motion Capture**: Here it means using 2D images to find the motion (kinematic instruction/joint parameters) to instruct a humanoid robot to mimic the human posture.

## Dataset generation

To build the synthetic dataset, first ensure that the cameras pointing to the humanoids are active in the `./config.sh`.

```
CAMERA_ENABLED_IDS='500 501 502 503'
```

and rebuild the project with these commands:

```
./control.sh build
```

The image below describes how the dataset generation system works.

![Dataset generation overview](resources/dataset_generation.jpg)

To create a humanoid dataset (paired pose data, motion data and reference images) in the `DATASET/TRAIN/` directory:

```bash
./control.sh dataset TRAIN/
./control.sh dataset EVAL/
```

To find the implementation of how the dataset is created, go to [pre_processing/](./pre_processing/) directory.

When the data is transformed into numpy/tabular format, each row in the CSV file represents **multiple pose** samples **from each camera** and its corresponding robot motion. The number of columns depends on the number of cameras, and it's possible to select what camera inputs you want to use; i.e., just one camera input or up to four camera inputs.

- The first columns are the 3D coordinates (x, y, z) for each of the 33 MediaPipe pose landmarks and **for each camera**, named like `cam500_0_x, cam501_0_x, cam502_0_x, cam503_0_x, cam500_0_y, cam501_0_y, cam502_0_y, cam503_0_y, cam500_0_z, cam501_0_z, cam502_0_z, cam503_0_z...`.
- The remaining columns are the robot joint positions (motion targets) for that sample, with names like jRightShoulder_rotx, jLeftElbow_roty, etc.

## Prebuild datasets

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

## Loading Dataset

For training and prediction, we have a helper library that loads the data into these two primitive numpy arrays. All training must use these two data arrays with the following shape:

```
motion_NP
-------------
scenario_id, motion_joint[1-47]
1, [JDATA_a]
2, [JDATA_b]

pose_NP
------------
scenario_id, cam_id, mp_landmark[1-99]
1, 500, [LDATA_a]
1, 501, [LDATA_b]
2, 500, [LDATA_c]
2, 501, [LDATA_d]
```

## Models

### PyTorch

We have more control over the PyTorch model. It has its own implementation of dataset and model definition, located inside [utils.py](./pose_to_motion/pytorch/utils.py). What is specific about using PyTorch is the possibility to use Optuna as a hyperparameter optimization tool. This tool helps find the best suitable set of hyperparameters given its training data. AutoGluon has its own internal optimization; thus we only use Optuna for PyTorch.

- Input Layer:  33 MediaPipe pose landmarks for each camera (x, y, z coordinates).
- Hidden Layers: Multi-layer perceptron with pose normalization preprocessing
- Output Layer: Robot joint position sequences for humanoid motion control, this results in a total of 47 joints to be outputted.
- Training: Supervised learning on pose-motion paired datasets.

### AutoGluon

We use [Autogluon tabular predictor ](https://auto.gluon.ai/stable/api/autogluon.tabular.TabularPredictor.html). This model is trained as an ensemble (collection) of models where each separate model aims to predict a single joint given the complete input poses. The result from each model is then merged together and becomes a predicted list for each target joint.

## Model Training

To train a model run the command below. The input size depends on the number of cameras you plan on using, meaning that if you only select one camera the input shape will take **1** camera into account. If you select **4** the model will have a **4 times** larger input size. To set it up in the way you want, go to [`config.sh`](config.sh). The variables are:

- `$model_instance`: If you want to reuse a model, specify its name here. Keep blank if you don't want to save.
- `$dataset_cameras`: The cameras you want to train on. This can be a list of cameras i.e. "500 501 502". For single training set this to one  ""
- `$model_type`: Possible selections: pytorch, autogluon

Finally you specify what JSON directory you want to use as training data which becomes the second argument, below is an example of running train using the TRAIN/ dataset.

After a training run is complete, the model is saved inside of `pose_to_motion/{$model_type}/output/saved_model_states`. A summary report of the session is also generated and will be saved inside `pose_to_motion/{$model_type}/output/train_report.csv`.
Ensure that the `./config.sh` is properly configured.

```
# The cameras you want to train on. This can be a list of cameras i.e. "500 501 502". For single training set this to one ""
dataset_cameras='500'

# Possible selections: pytorch, autogluon
model_type="pytorch"

# If you want to reuse a model, specify its name here. Keep blank if you don't want to save.
model_instance="pytorch_test_pred_500"

```

> Keep in mind that we use a separate virtual environment to install machine learning related pip packages called `mlenv` with a separate [`requirements.txt`](pose_to_motion/MLrequirements.txt). Build the environment first using `./control.sh mlenv`

```

./control.sh mlenv
./control.sh train TRAIN/

```

## Model Evaluation

Running `control.sh eval` in the terminal runs the evaluation pipeline. This will trigger the selected `model_type`'s evaluation pipeline. To set it up in the way you want, go to [`config.sh`](config.sh). The variables are:

- `$model_instance`: If you want to reuse a model, specify its name here. Keep blank if you don't want to save.
- `$dataset_cameras`: The cameras you want to eval on. This can be a list of cameras i.e. "500 501 502". For single training set this to one  ""
- `$model_type`: Possible selections: pytorch, autogluon

The evaluation will run metrics defined in [metrics.py](./pose_to_motion/metrics.py); currently MSE and MAE are calculated. The results are then saved in a summary report located in `pose_to_motion/{$model_type}/outputs/eval_report.csv`.

To evaluate a trained model, there is a command that evaluates based on a set of metric. In the example below it evaluates on the EVAL/ JSON-dataset:

```

./control.sh eval EVAL/

```

As the result MSE and MAE values are printed.

## Model Prediction

The input format needs to be structured as the template below. Please note the "dataset_name" can be whatever you want to describe the content of the data. The camera names are connected with the variable `dataset_cameras` inside [config.sh](/config.sh). Lastly, this folder needs to be placed inside the [input/](input/) directory.

```
DATASET_NAME_TEMPLATE_SOME_DESCRIPTOR/
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

The prediction pipeline uses these variables in the `config.sh`:

- `humanoid_input_dir`: Input directory. Images or videos in this dir can be predicted.
- `humanoid_output_dir`: Output directory. pose files and predicted motions are placed here
- `dataset_cameras`: What cameras to take into concern for prediction.
- `model_instance`: What trained model instance to use.
- `replay_motion_namespace`: What humanoid you want to replay motions on.

The prediction pipeline is implemented in a way where it expects the input dataset to follow the exact format as the model was trained on; see the note above for reference. It is possible to predict on three different types of data: `images`, `videos`, and raw `JSON` data. A good folder to put the data is inside the `input/` directory.

The input dataset gets processed into JSON format and is saved inside the `output/` directory, using `processes_input_data.py`. It creates the pose json files and MediaPipe images. `predict.py` inputs the pose json files and generates the predicted motions by saving them into a directory called `$humanoid_output_dir/dataset_name/predicted_motions/`. Running `control.sh predict $mode $dataset_name` inputs all poses for each camera in JSON format and for each pose_data outputs a predicted motion, which is replayable in RViz along with a ground truth pose_image. Finally, all motions are inputted sequentially into the motion viewer and visualized inside of Gazebo.

To run predict successfully, ensure that the variables in `config.sh` are set and place the data you want to predict inside `$humanoid_input_dir`. Then use the folder name of the dataset you want to predict as an argument along with the format of the data.

For testing make sure that

- `input/TEST` (with DATASET_NAME_TEMPLATE_SOME_DESCRIPTOR explained above) is present (**`input/` and not `DATASET/`**)
- `input/VIDEOS/camera_500/20250611video.mp4`

To apply the predicted motion to your preferred humanoid adjust the `config.sh`:

```bash
replay_motion_namespace="humanoid_2"
dataset_cameras='500'
```

Finally run the command below:

```
./control.sh predict TEST/
./control.sh predict VIDEOS/
./control.sh predict IMAGES/
```

## Project Structure:

- [`humanoid_ml_requirements.txt`](humanoid_ml_requirements.txt): Contains the humanoid machine learning requirements

- [`input/`](input) : This folder has the pose data to be predicted

- [`output/`](output) : This folder saves the predicted motion data and intermediate results

- [`/DATASET/TRAIN`](DATASET/TRAIN): Contains motion, pose and image files generated by `./control.sh dataset TRAIN`

  - `motion_data` : Corresponding random motion (request)
  - `camera/pose_data` : Mediapipe pose (result) for each camera
  - `camera/pose_images`: Mediapipe annotated images and original images for each camera

- [`/DATASET/EVAL`](DATASET/EVAL): Contains motion, pose and image files generated by `./control.sh dataset EVAL`

  - `motion_data` : Corresponding random motion (request)
  - `camera/pose_data` : Mediapipe pose (result) for each camera
  - `camera/pose_images`: Mediapipe annotated images and original images for each camera

- [`pre_processing/`](pre_processing): contains all pre-processing files, used to convert the JSON data into tabular data. This will be the input of the model.

  - [`process_input_data.py`](process_input_data.py): Handles pose detection from actual images or videos using MediaPipe
  - [`dataframe_converter.py`](dataframe_converter.py): Contains helper functions to convert json files into dataframe/numpy format and is used by the models. Running its main will generate csv files that contains both pose and motion data from the json files. Inputs the cameras you want to use, the json dir, the csv filename to generate.

- [`pose_to_motion/`](pose_to_motion): Contains the ML training pipeline for pose-to-motion prediction

  - [`model.py`](model.py): The main file containing implementations for training, evaluating, and predicting motions. This file acts as an orchestrator and inputs the data, what model to use, and what operation should be done. It is responsible for handling commands.

  - [`autogluon/`](pose_to_motion/autogluon): directory containing model implementation for autogluon model. Handles training, evaluation, and predicting motions.

    - [`framework.py`](pose_to_motion/autogluon/model.py): Framework file, contains the implementation for train, evaluation, and prediction.
    - [`utils.py`](pose_to_motion/autogluon/utils.py): Utils file, contains helper methods, and model definition. The `model.py` use this.
    - [`output/`](pose_to_motion/autogluon/output): Contain reports of ran sessions and saved model states. Saved models will be found here
      - [`saved_model_states/`](pose_to_motion/autogluon/output/saved_model_states): Saved autogluon models are stored here
      - [`reports/`](pose_to_motion/autogluon/output/reports): Manually generated reports file containing training/evaluation run information. Good for reproducibility. The reports are generated by `generate_report.py` and the new report row is saved in `train_report.csv` or `eval_report.csv`

  - [`pytorch/`](pose_to_motion/pytorch): directory containing model implementation for pytorch model. Handles training, evaluation, and predicting motions.

    - [`framework.py`](pose_to_motion/pytorch/model.py): Framework file, contains the implementation for train, evaluation, and prediction.
    - [`utils.py`](pose_to_motion/pytorch/utils.py): Utils file, contains helper methods, and model definition. The `model.py` use this.
    - [`output/`](pose_to_motion/pytorch/output): Contain reports of ran sessions and saved model states. Saved models will be found here
      - [`saved_model_states/`](pose_to_motion/pytorch/output/saved_model_states): Saved autogluon models are stored here
      - [`reports/`](pose_to_motion/pytorch/output/reports): Manually generated reports file containing training/evaluation run information. Good for reproducibility. The reports are generated by `generate_report.py` and the new report row is saved in `train_report.csv` or `eval_report.csv`
      - [`optuna_studies/`](pose_to_motion/pytorch/output/optuna_studies): Specific for pytorch. Optuna studies are stored at this folder. See [optuna section](#optuna) for more information about Optuna.

  - [`generate_report.py`](pose_to_motion/generate_report.py):  Contains methods to generate report files, used after a training or evaluation session has been done for any selected model. These generated reports contain information for reproducing runs and how a model performed against metrics. A new report is generated every time we run train or eval on a model and every report is saved in a single CSV file where every row is a report. The files are named: `train_report.csv` or `eval_report.csv`.

  - [`metrics.py`](pose_to_motion/metrics.py): Contains all functions for calculating metrics for models for any selected model.

  - [`humanoid_config.py`](pose_to_motion/humanoid_config.py): it contains the humanoid configuration like the number of joints and the number of pose landmarks.

## Send motion commands

The package [simulation/random_motion_planner/](/simulation/random_motion_planner/) allows us to execute motions to any humanoid robot using the MoveIt2 framework. It is set up in config.sh where you set the variable below, which is the robot namespace you want to control.

```
replay_motion_namespace="humanoid_1"
```

This package has two features. You can run it to execute a series of random motions which is useful when you want to create synthetic data. The other feature is that when the package is running for a given humanoid robot, you can send motions you want to execute directly. When the package runs it subscribes to a topic: `humanoid_X/execute_motion` which inputs a stringified message of a dict (joint_name, value).

The easiest way to use it is in python using a dictionary, stringifying it with json, and then publishing it to the topic:

```
motion_dict = {JOINT_DATA}
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
```

## Model structure

`model.py` is responsible for routing the training and prediction to the right model. When running `model.py train modelX=pytorch_model cams=[500,501]` from `./control.sh`, the pseudocode below in `model.py` loads the right model and passes the needed information for training and prediction to the model.

```
import modelX
modelX.train(pose_NP, motion_NP, cams)
motion = modelX.train(pose_NP, cams)
pred= modelX.train(pose_NP, cams)
```

modelX can now have all information that it needs for training or prediction. Each model has to define at least the following interfaces:

```
modelX.py:

def modelX.train():
    FL_POSE  = FLATTEN_POSE_BY_SCENARIO(pose_NP, cams)
    // 1, [LDATA_a], [LDATA_b]
    x = join(FL_POSE, motion_NP)
    // [LDATA_a], [LDATA_b], [JDATA_a]
    train with the data above
    SAVE_MODEL_AS(str(cams))

def modelX.predict(pose_NP, cams):
    FL_POSE  = FLATTEN_POSE_BY_SCENARIO(pose_NP, cams)
    LOAD_MODEL(str(cams), FL_POSE)
    WHATEVER
    return motion_np
```

![](/resources/diagrams/humanoid_mocap_pipeline.drawio.png)

## The Theory

- `I` : an image. (`I_s`: from simulator, `I_r` from real world)
- `P` : a pose (mediapipe output)
- `M` : a motion (moveit2)

then

- `SIM(M) -> I` : Simulator(gazebo) using inverse kinematics(moveit2) to convert the motion `M` to create image `I`
- `PE(I) -> P` : Pose estimator(mediapipe) takes the image `I`, to find human pose `P`
- `Q(P) -> M` : machine learning model `Q`, takes pose `P` and tries to replicate that pose estimator(mediapipe) pose using motion (moveit) `M`

Assumption: Pose estimator(mediapipe) performs good enough that it can detect human poses from both simulator and real-world domains:

- `PE(I_r) -> P_r`
- `PE(I_s) -> P_s`

Our goal is to

- pass random M to build the dataset of pairs :  `<M,PE(SIM(M))> = <M,P>`
- Use the dataset above to find `Q()` which is the inverse of this `PE(SIM())`

Now we can do motion capture (replicate real human movements) by `Q(PE(I_r))`
