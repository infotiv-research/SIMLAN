# Pytorch

## Hyperparameters:

Currently the model hyperparameters (`HyperparameterDict`) in `humanoid_utility/pose_to_motion/pytorch/utils.py` are tuned using Optuna. It performs
train/validation splitting, reports validation MSE to the study and uses a
pruner to stop unpromising trials early.

In order to use Optuna, you need to activate the ml environment.

```
source ~/mlenv/bin/activate
python /home/ros/src/humanoid_utility/pose_to_motion/pytorch/optuna_tuner.py --csv DATASET.csv --n-trials 50 --n-jobs 1
```

## Overfitting Control (Single-Camera)

### Neural Network Regularization

- weight_decay (L2)
  It prevent overfitting by penalizing large weight values. So it controls overfitting by ensuring the model doesn’t focus too heavily on patterns that might not generalize.
- Lost function
  SmoothL1Loss(beta=0.1) is appropriate for multi-joint regression: smooth for small errors, robust to outliers and stabilize training.
- Early stopping
  Training stops when validation error no longer improves.

### Preprocessing

- Root Centering (Pelvis-based)
  Human poses are made translation-invariant by centering on the pelvis.
- Pelvis = midpoint of left hip (23) and right hip (24)
- All joints expressed relative to pelvis
  This removes global position and keeps body configuration only.

### Normalization

- Features and labels are standardized using Z-score normalization computed from the training set only.
- During evaluation/prediction, the same normalization is applied using stored training statistics.
- Denormalization is applied after model output to compute real-world metrics (MSE, MAE, max joint error).

This ensures:

- stable gradients
- comparable feature scales
- no data leakage

### Data Augmentation (Pose Space)

Augmentation is applied after normalization during training only. The goal is to simulate realistic pose estimation noise.

- Gaussian jitter (motion caption noise)
  Adds small random noise to all joints except pelvis.
- Joint dropout (simulating missing detections)
  Randomly replaces joints with small noise. Pelvis is never dropped.
- Limb dropout (rare event)
  Randomly selects a limb (arm or leg) and replaces all its joints with small noise. It helps model robustness to occlusions or tracking failures.
