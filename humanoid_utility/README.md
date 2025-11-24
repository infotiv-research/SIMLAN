# Humanoid Motion Capture

This project develops a system for translating human pose detection to humanoid robot motion in simulation environments. Using Google MediaPipe for pose landmark detection from camera input, the system maps detected human poses to corresponding joint movements executed by a humanoid robot in Gazebo simulator. The implementation leverages ROS2 Ignition and MoveIt2 for motion planning and control, with a data generation pipeline that creates training pairs of pose landmarks and robot joint configurations. This approach provides a foundation for safety monitoring applications in industrial simulation (SIMLAN), where human pose analysis can be integrated for workplace incident detection. The work is based on master thesis "Human Motion Replay on a Simulated Humanoid Robot Using Pose Estimation" by Tove Casparsson and Siyu Yi, Supervised by Hamid Ebadi, June 2025.

![Pose translation](resources/pose_translation.jpg)


## Terminology:


- **Forward Kinematics (FK)**: The process of calculating the position and orientation of a robot's links given the values of its joint parameters (e.g., angles or displacements). In other words, FK answers the question: "Where is the robot's hand if I know all the joint values? (usually have one answer)"
- **Inverse Kinematics (IK)**: The process of determining the joint parameters (e.g., angles or displacements) required to achieve a desired position and orientation of the robot's links. In other words, IK answers the question: "What joint values will place the robot’s hand here? (usually have many answers)"
- **Pose** : 3D pose landmarks (MediaPipe) extracted by Mediapipe from an 2D image of human posture
- **Motion** : A kinematic instruction (joint parameters) sent to the robot (via MoveIt) for execution. While "kinematic instruction" would be a more accurate term, we continue to use "motion" for historical reasons (used in the word motion-capture), even though , "motion" often refers to the difference/movement between two postures (e.g., posture2 - posture1).
- **Motion Capture**: Here it means using 2D images to find the motion(kinematic instruction/joint parameters) to instruct a humanoid robot to mimic the human posture.


## Dataset

To find the implementation of how the dataset is created, go to [pre_prossessing/ directory](./pre_processing/). We have two dataformats for multi and single camera prediction. 

For the single-camera dataset, each row in the CSV file represents **one pose** sample and its corresponding robot motion from a specific camera.
- The first columns are the 3D coordinates (x, y, z) for each of the 33 MediaPipe pose landmarks, named like `cam500_0_x, cam500_0_y, cam500_0_z,...`.
- The remaining columns are the robot joint positions (motion targets) for that sample, with names like `jRightShoulder_rotx`, `jLeftElbow_roty`, etc.

For the multi-camera dataset, each row in the CSV file represents **multiple poses** samples **from each camera** and its corresponding robot motion.
- the first columns are the 3D coordinates (x, y, z) for each of the 33 MediaPipe pose landmarks and **for each camera**, named like `cam500_0_x, cam501_0_x, cam502_0_x, cam503_0_x, cam500_0_y, cam501_0_y, cam502_0_y, cam503_0_y, cam500_0_z, cam501_0_z, cam502_0_z, cam503_0_z...`.
- The remaining columns are the robot joint positions (motion targets) for that sample, with names like jRightShoulder_rotx, jLeftElbow_roty, etc.


### Translation of a detected pose to humanoid motion command​

This project employs a deep neural network to learn the mapping between human pose and humanoid robot motion. The model takes 33 MediaPipe pose landmarks as input and predicts corresponding robot joint positions.



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

For training, prediction we have helper library that loads the data to these two primitive numpy arrays. All training has to use these two data array with the following shape: 

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

## Model structure
model.py is responsible to route the training and prediction to the right model.
when running `model.py train modelX=pytorch_model cams=[500,501]` from ./`control.sh` , the psuedocode below in `model.py` loads the right model and pass the needed information for training and prediction to the model.

```
import modelX
modelX.train(pose_NP, motion_NP, cams)
motion = modelX.train(pose_NP, cams)
pred= modelX.train(pose_NP, cams)
```

modelX can now has all information that it needs for training or prediction. Each model has to define at least the following interfaces:

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

### Neural network design

- Input Layer:  33 MediaPipe pose landmarks for each camera (x, y, z coordinates). 
- Hidden Layers: Multi-layer perceptron with pose normalization preprocessing
- Output Layer: Robot joint position sequences for humanoid motion control, this results in a total of 47 joints to be outputed.
- Training: Supervised learning on pose-motion paired datasets. 

## Project Structure:
- [`humanoid_ml_requirements.txt`](humanoid_ml_requirements.txt): Contains the humanoid machine learning requirements
- [`input/`](input) : This folder has the pose data to be predicted
- [`output/`](output) : This folder saves the predicted motion data and intermediate results
- [`/DATASET_RAW/TRAIN`](DATASET_RAW/TRAIN): Contains motion, pose and image files generated by `./control.sh dataset TRAIN`
  - `motion_data` : Corresponding random motion (request)
  - `camera/pose_data` : Mediapipe pose (result) for each camera
  - `camera/pose_images`: Mediapipe annotated images and original images for each camera
- [`/DATASET_RAW/EVAL`](DATASET_RAW/EVAL): Contains motion, pose and image files generated by `./control.sh dataset EVAL`
  - `motion_data` : Corresponding random motion (request)
  - `camera/pose_data` : Mediapipe pose (result) for each camera
  - `camera/pose_images`: Mediapipe annotated images and original images for each camera
- [`pre_processing/`](pre_processing): contains all pre-processing files, used to convert the JSON data into tabular data. This will be the input of the model.
  - [`mp_detection.py`](mp_detection.py): Handles pose detection from actual images or videos using MediaPipe
  - [`load_json_data.py`](load_json_data.py): Build a csv file that contains both pose and motion data from the json files and outputs a default csv format.  
  - [`csvtowide.py`](csvtowide.py): Convert the default csv format into single and -multi camera csv files. (this data is the actual model input). 
  
- [`pose_to_motion/`](pose_to_motion): Contains the ML training pipeline for pose-to-motion prediction
  - [`autogluon/`](pose_to_motion/autogluon): directory containing model implementation for autogluon model. Handles training, evaluation, and predicting motions.
    - [`model.py`](pose_to_motion/autogluon/model.py): Main file, contains the implementation for train, evaluation, and prediction.
    - [`utils.py`](pose_to_motion/autogluon/utils.py): Utils file, contains helper methods, and model definition. The `model.py` use this.
    - [`output/`](pose_to_motion/autogluon/output): Contain reports of ran sessions and saved model states. Saved models will be found here 
      - [`saved_model_states/`](pose_to_motion/autogluon/output/saved_model_states): Saved autogluon models are stored here  
      - [`reports/`](pose_to_motion/autogluon/output/reports): Manually generated reports file containing training/evaluation run information. Good for reprodusability. The reports are generated by `generate_report.py` and the new report row is saved in `train_report.csv` or `eval_report.csv`

  - [`pytorch/`](pose_to_motion/pytorch): directory containing model implementation for pytorch model. Handles training, evaluation, and predicting motions.
    - [`model.py`](pose_to_motion/pytorch/model.py): Main file, contains the implementation for train, evaluation, and prediction.
    - [`utils.py`](pose_to_motion/pytorch/utils.py): Utils file, contains helper methods, and model definition. The `model.py` use this.
    - [`output/`](pose_to_motion/pytorch/output): Contain reports of ran sessions and saved model states. Saved models will be found here 
      - [`saved_model_states/`](pose_to_motion/pytorch/output/saved_model_states): Saved autogluon models are stored here  
      - [`reports/`](pose_to_motion/pytorch/output/reports): Manually generated reports file containing training/evaluation run information. Good for reprodusability. The reports are generated by `generate_report.py` and the new report row is saved in `train_report.csv` or `eval_report.csv`
      - [`optuna_studies/`](pose_to_motion/pytorch/output/optuna_studies): Specific for pytorch. Optuna studies are stored at this folder. See [optuna section](#optuna 
 ) for more information about Optuna.
  - [`generate_report.py`](pose_to_motion/generate_report.py):  Contains methods to generate report files, used after a training or evaluation session has been done for any selected model. These generated reports contain information for reproducing runs and how a model performed against metrics. A new report is generated every time we run train or eval on a model and every report is saved in a single CSV file where every row is a report. The files are named: `train_report.csv` or `eval_report.csv`.
  - [`metrics.py`](pose_to_motion/metrics.py): Contains all functions for calculating metrics for models for any selected model. 
  - [`humanoid_config.py`](pose_to_motion/humanoid_config.py): it contains the humanoid configuration like the number of joints and the number of pose landmarks.



## Dataset generation


The below image describes how the dataset generation system works.

![Dataset generation overview](resources/dataset_generation.jpg)


To create a humanoid dataset ( paired pose data, motion data and reference images) in the `DATASET_RAW/TRAIN` directory:

```bash
./control.sh dataset TRAIN/
```

## Dataset preprocessing


The diagram below shows the overall workflow of this project. Pose and motion data are first collected from JSON files and combined into a single CSV file. These processed csv files are ready to be fed into a model of your choice, Autogluon or pytorch.  See below to find more information about the dataset or more about the [models](#models). 


![Overall workflow](resources/ML.drawio.png)



> The dataset, input and outputs are in `humanoid_utility/DATASET_RAW/` directory.


The resulting files are stored as parallel `.json` files in multi-camera folders `camera_*/pose_data`, `camera_*/motion_data`, `camera_*/pose_images`.
To avoid creation of `pose_images` that are only used as the ground truth and debugging (specially if you are building your training data), comment out then call to `self.save_pose_image()` in `camera_viewer.py`.

Finally to merge them all in tabular `.csv` file run the following command.
```
./control.sh convert2csv
```
Results in merging:

- `DATASET_PROCESSED/train.csv` to `DATASET_PROCESSED/TRAIN/single_train_cam501.csv`
- `DATASET_PROCESSED/train.csv` to `DATASET_PROCESSED/TRAIN/single_train_cam502.csv`
- `DATASET_PROCESSED/train.csv` to `DATASET_PROCESSED/TRAIN/single_train_cam500.csv`
- `DATASET_PROCESSED/train.csv` to `DATASET_PROCESSED/TRAIN/single_train_cam503.csv`
- `DATASET_PROCESSED/train.csv` to `DATASET_PROCESSED/TRAIN/multi_train.csv`
- `DATASET_PROCESSED/eval.csv` to `DATASET_PROCESSED/EVAL/single_eval_cam500.csv`
- `DATASET_PROCESSED/eval.csv` to `DATASET_PROCESSED/EVAL/single_eval_cam501.csv`
- `DATASET_PROCESSED/eval.csv` to `DATASET_PROCESSED/EVAL/single_eval_cam502.csv`
- `DATASET_PROCESSED/eval.csv` to `DATASET_PROCESSED/EVAL/single_eval_cam503.csv`
- `DATASET_PROCESSED/eval.csv` to `DATASET_PROCESSED/EVAL/multi_eval.csv`
  
You can replay_motion each motion data separately: `./control.sh replay_motion DATASET_PROCESSED/EVAL/motion_data/AAAAAAA_motion.json`


## Model Training

### Autogluon

We use [Autogluon tabular predictor ](https://auto.gluon.ai/stable/api/autogluon.tabular.TabularPredictor.html). This model is trained as an ensemble (collection) of models where each separate model aims to predict a single joint given the complete input poses. The result from each model is then merged together and becomes a predicted list for each target joint. 

### Pytorch

We have more control over the Pytorch model. It has its own implementation of its dataset and model definition, located inside of [its utils dir](./pose_to_motion/pytorch/utils.py) folder. What is specific about using the pytorch is the possibility to use optuna as a hyperparameter optimization tool. This tools help find the best suitable set of hyperparams given its training data. Autogluon has its own internal optimization thus we only use this for pytorch. 

## Optuna 

Optuna is a hyperparameter optimization framework available as a python library. It acts as a wrapper around an existing training framework where it uses the same setup of dataset, train_loop and hyperparameters, so it has the same context as when you would run it yourself. Optuna starts by defining an objective function. The goal of the objective function is either to minimize or maximize a value. You specify this value yourself, for this setup we have typically used the validation MSE to use as a goal to minimize. We also need to define the scope of values for the hyperparameters. We define for every hyperparameter, a range of values which Optuna is allowed to select from. i.e. BATCHSIZE being in range(4,128). When we have done this for every param we want to find the optimal value for, we start the start optuna by running a "study". Optuna runs N number of training sessions, each independent and uses internal algorithms to find the best set of values for the list of hyperparameters. In the end we get the training session that scored the best and can use its hyperparameters.  

Note: We only use this for pytorch since autogluon has its own hyperparameter optimization. This is why we only see studies inside `pytorch/optuna_studies/`. 

### Commands

> Keep in mind that we use a separate virtual environment to install machine learning related pip packages called `mlenv`  with a separate [`requirements.txt`](pose_to_motion/MLrequirements.txt). Build the environment first using `./control.sh mlenv`

Running `control.sh single_train` or `control.sh multi_train` in the terminal runs the training pipeline. Single train expects data from 1 camera source whereas multi train can use unlimited amount of camera input. Both models use 1 target set of joints. 

 The interface to modify the runtime arguments, you modify variables inside of `config.sh`. The runtime variables you can change are these:
- **model_type**. Possible selections: pytorch, autogluon
- **pose_to_motion_model_name**. If you want to reuse a model, specify its name here. Keep blank if you dont want to save.
- **pose_to_motion_session_name**. Optional if you want to describe your session.

After a training run is complete, the model is saved inside of `pose_to_motion/{$model_type}/output/saved_model_states`. A summary report of the session is also generated and will be saved inside `pose_to_motion/{$model_type}/output/train_report.csv`


There are two options for using the data and training a model. The resulting model is saved in `{$model_type}/output/saved_model_states`. Training a model using 1 camera source, reads data from `DATASET_PROCESSED/TRAIN/single_train_cam5XX.csv` and trains our model.

```
./control.sh single_train
```

Training a model using multiple camera sources, reads data from `DATASET_PROCESSED/TRAIN/multi_train.csv` and trains our model:

```
./control.sh multi_train
```


## Model Evaluation 

Running `control.sh eval` in the terminal runs the evaluation pipeline. This will trigger the selected `model_type`'s evaluation pipeline. The same run time variables found in `config.sh`, used for training is used for evaluation where the key variables **model_type** and **pose_to_motion_model_name**. 

The evaluation will run metrics defined inside of [metrics.py](./pose_to_motion/metrics.py), currently MSE and MAE will be calculated. The results is then saved in a summary report localted in `pose_to_motion/{$model_type}/outputs/eval_report.csv`. 


To evaluate a trained model, there is a command that evaluates based on a set of metric:

```
./control.sh eval
```

## Model Prediction

Running `control.sh predict` will run input all poses for each camera, in JSON format and for each pose_data, output a predicted motion, which is replayable in RViz along with a ground truth pose_image.   


The following command uses MediaPipe to create pose data from an image or a video in `input/` to populate `output/` with mediapipe landmark pose and images.


```
./control.sh image_pipeline
./control.sh video_pipeline
```
To use the model on each generated pose, run the following command to generate predicted motions which will be saved inside `predict_data/`, inside of the input folder. 

```
./control.sh single_prediction output
```

To use the model on each pose inside of any `DATASET_RAW` directory for example: `DATASET_RAW/EVAL/camera_500/pose_data`, run the following command below for single camera using the /EVAL dataset and camera 500 as input:

```
./control.sh single_prediction DATASET_RAW/EVAL
```

This will save the predicted motions to `DATASET_RAW/EVAL/predict_data`. Afterwards it opens up a image viewer with the ground truth mediapipe detection as well to quickly check the performance of our ml model:

To use it for multi camera use the command below to predict motions. This works the same as `single_prediction` but instead use multi-camera datasets.
```
./control.sh multi_prediction DATASET_RAW/EVAL
```



## Misc. (IGNORE WHAT COMES NEXT)

### Notes
- It is a "feature" (not a bug) that some cameras cannot detect the pose (NaN values). It helps to be still able to detect pose when a part of body is masked.
- Mediapipe cannot reliable detect the depth, back and front
- Experiment different options for NaN values (average of other rows, zero, etc)

# Preprocessing

![pose_landmarks_index.png](resources/pose_landmarks_index.png)

https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker

Three steps of normalization (feature engineer) of mediapipe landmarks:
- Position normalization
- Scale normalization
- Rotation normalization


![normalization.png](resources/Normalisation.drawio.png)





## Issues and Future Works

### Current Performance Metrics
The upgraded multi-camera system demonstrates the following evaluation results:
- **Mean Squared Error (MSE)**: 0.206
- **Root Mean Squared Error (RMSE)**: 0.454

### Known Limitations
- MediaPipe exhibits difficulty distinguishing between front and back orientations, leading to potential left/right body side confusion in pose detection
- Current preprocessing includes normalization for single-camera data (using hip midpoint as origin), but multi-camera normalization requires further development
- Depth estimation reliability remains limited in MediaPipe's 2D-to-3D pose conversion

### Future Development Areas
- Enhance multi-camera data preprocessing with normalization, also probably replace the `pose_landmarks` into `pose_world_landmarks` in `camera_viewer.py` 
- Replace pre-processing with pose-processing, and pay attention to whether Mediapipe's `static mode` is enabled.
- Consider applying feature engineering to enable the model to gain a deeper and more accurate understanding of the data. 



## The Theory

- 
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

No we can do motion capture (replicate real human movements) by `Q(PE(I_r))` 
