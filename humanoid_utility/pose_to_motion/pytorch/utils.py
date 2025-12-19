import sys
from typing import List, TypedDict

from humanoid_utility import humanoid_config

sys.path.append(".")
import torch
from torch.utils.data import Dataset
import torch.nn as nn
from pathlib import Path
import random


MODEL_STATES_DIR = "humanoid_utility/pose_to_motion/pytorch/output/saved_model_states"


class HyperparameterDict(TypedDict):
    # TRAIN LOOP SPECIFIC
    N_EPOCHS: int
    BATCHSIZE: int
    optimizer_name: str
    lr: float
    n_layers: int
    # MODEL SPECIFIC
    size_per_layer: List[int]
    dropout_per_layer: List[float]
    in_features: int
    out_features: int


def get_hyperparameters(num_cameras) -> HyperparameterDict:
    return HyperparameterDict(
        {
            # TRAIN LOOP SPECIFIC
            "N_EPOCHS": 160,
            "BATCHSIZE": 22,
            "optimizer_name": "Adam",
            "lr": 7.088648070427078e-05,
            "n_layers": 3,
            # MODEL SPECIFIC
            "size_per_layer": [504, 476, 479],
            "dropout_per_layer": [
                0.13201141925241577,
                0.20817940982863983,
                0.14885002648956877,
            ],
            "in_features": humanoid_config.NUM_MARKERS * 3 * num_cameras,
            "out_features": humanoid_config.NUM_JOINTS,
        }
    )


class MultiLabelRegressionDataset(Dataset):
    def __init__(self, dataset_df, cameras):
        cam_pose_cols = [
            f"cam{camera}_{pose_col}"
            for camera in cameras
            for pose_col in humanoid_config.pose_names
        ]
        self.features = torch.tensor(
            dataset_df[cam_pose_cols].values, dtype=torch.float32
        )  # first column is index, then features
        self.labels = torch.tensor(
            dataset_df[humanoid_config.movable_joint_names].values, dtype=torch.float32
        )  # Last num_feat columns as labels

    def __len__(self):
        return len(self.features)

    def __getitem__(self, idx):
        return self.features[idx], self.labels[idx]


def define_torch_model(hyperparameters: HyperparameterDict):
    # We optimize the number of layers, hidden units and dropout ratio in each layer.
    layers = []
    n_layers = hyperparameters["n_layers"]
    size_per_layer = hyperparameters["size_per_layer"]
    dropout_per_layer = hyperparameters["dropout_per_layer"]
    IN_FEATURES = hyperparameters["in_features"]
    OUT_CLASSES = hyperparameters["out_features"]

    in_features = IN_FEATURES
    for i in range(n_layers):
        out_features = size_per_layer[i]
        layers.append(nn.Linear(in_features, out_features))
        layers.append(nn.ReLU())
        p = dropout_per_layer[i]
        layers.append(nn.Dropout(p))

        in_features = out_features
    layers.append(nn.Linear(out_features, OUT_CLASSES))
    return nn.Sequential(*layers)


def save_model(model, model_instance):
    # if we run an optuna session, we just move on.
    if model_instance == "optuna":
        return
    print(f"Saving model: {model_instance}")

    torch.save(model.state_dict(), f"{MODEL_STATES_DIR}/{model_instance}.pth")
    return model_instance


def load_model(model_instance, hyperparameters):

    # We define the model
    model = define_torch_model(hyperparameters)
    # If model exist we load it. Else return new model
    if model_exist(model_instance):
        print(f"model found. Using saved model {model_instance}")
        try:
            model.load_state_dict(
                torch.load(f"{MODEL_STATES_DIR}/{model_instance}.pth")
            )
        except Exception as e:
            print(
                f"Unable to load model with name {model_instance}. Most probably the hyperparameters miss match."
            )
            print(e)

    else:
        print("No model with this name found. Using new model")
    return model


def model_exist(model_instance):
    # We check if file exist.
    my_file = Path(f"{MODEL_STATES_DIR}/{model_instance}.pth")
    return my_file.exists()
