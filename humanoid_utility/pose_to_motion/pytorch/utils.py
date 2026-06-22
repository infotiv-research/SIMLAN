import sys
from typing import List, TypedDict
import pickle

from humanoid_utility import humanoid_config

sys.path.append(".")
import torch
from torch.utils.data import Dataset
import torch.nn as nn
from pathlib import Path


MODEL_STATES_DIR = "humanoid_utility/pose_to_motion/pytorch/output/saved_model_states"


class HyperparameterDict(TypedDict):
    N_EPOCHS: int
    BATCHSIZE: int
    optimizer_name: str
    lr: float
    weight_decay: float
    n_layers: int
    size_per_layer: List[int]
    dropout_per_layer: List[float]
    activation: str
    use_batchnorm: bool
    in_features: int
    out_features: int


def get_hyperparameters(num_cameras) -> HyperparameterDict:
    return {
        "N_EPOCHS": 29,
        "BATCHSIZE": 64,
        "optimizer_name": "Adam",
        "lr": 0.00017991487419730356,
        "weight_decay": 0.00026199125598775464,
        "n_layers": 3,
        "size_per_layer": [455, 311, 478],
        "dropout_per_layer": [
            0.05522710898901941,
            0.12467372316195538,
            0.24141357228329585,
        ],
        "activation": "leaky_relu",
        "use_batchnorm": False,
        "in_features": humanoid_config.NUM_MARKERS * 3 * num_cameras,
        "out_features": humanoid_config.NUM_JOINTS,
    }


class MultiLabelRegressionDataset(Dataset):
    def __init__(self, dataset_df, cameras):
        cam_pose_cols = [f"cam{camera}_{pose_col}" for camera in cameras for pose_col in humanoid_config.pose_names]
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
    activation_name = hyperparameters.get("activation", "leaky_relu")
    use_batchnorm = hyperparameters.get("use_batchnorm", True)
    # Map activation name to activation function
    activation_map = {
        "leaky_relu": nn.LeakyReLU(),
        "relu": nn.ReLU(),
        "tanh": nn.Tanh(),
    }
    activation_fn = activation_map.get(activation_name, nn.LeakyReLU())
    in_features = IN_FEATURES
    for i in range(n_layers):
        out_features = size_per_layer[i]
        layers.append(nn.Linear(in_features, out_features))
        if use_batchnorm:
            layers.append(nn.BatchNorm1d(out_features))
        layers.append(activation_fn)
        p = dropout_per_layer[i]
        if p > 0:
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
            model.load_state_dict(torch.load(f"{MODEL_STATES_DIR}/{model_instance}.pth"))
        except RuntimeError as e:
            print(f"Unable to load model with name {model_instance}. Most probably the hyperparameters miss match.")
            print(e)

    else:
        print("No model with this name found. Using new model")
    return model


def model_exist(model_instance):
    # We check if file exist.
    my_file = Path(f"{MODEL_STATES_DIR}/{model_instance}.pth")
    return my_file.exists()


def save_normalization_stats(feature_mean, feature_std, label_mean, label_std, model_instance):
    """Save normalization statistics"""
    if model_instance == "optuna":
        return
    print(f"Saving normalization stats for: {model_instance}")

    # Convert tensors to CPU if needed and store as numpy for persistence
    stats = {
        "feature_mean": feature_mean.cpu().numpy() if isinstance(feature_mean, torch.Tensor) else feature_mean,
        "feature_std": feature_std.cpu().numpy() if isinstance(feature_std, torch.Tensor) else feature_std,
        "label_mean": label_mean.cpu().numpy() if isinstance(label_mean, torch.Tensor) else label_mean,
        "label_std": label_std.cpu().numpy() if isinstance(label_std, torch.Tensor) else label_std,
    }

    stats_path = Path(f"{MODEL_STATES_DIR}/{model_instance}_stats.pkl")
    with open(stats_path, "wb") as f:
        pickle.dump(stats, f)


def load_normalization_stats(model_instance):
    """Load normalization statistics if it exists, else return None"""
    stats_path = Path(f"{MODEL_STATES_DIR}/{model_instance}_stats.pkl")

    if not stats_path.exists():
        print(f"No normalization stats found for {model_instance}")
        return None

    print(f"Loading normalization stats for: {model_instance}")
    with open(stats_path, "rb") as f:
        stats_np = pickle.load(f)

    # Convert numpy arrays back to tensors
    stats = {
        "feature_mean": torch.tensor(stats_np["feature_mean"], dtype=torch.float32),
        "feature_std": torch.tensor(stats_np["feature_std"], dtype=torch.float32),
        "label_mean": torch.tensor(stats_np["label_mean"], dtype=torch.float32),
        "label_std": torch.tensor(stats_np["label_std"], dtype=torch.float32),
    }
    return stats
