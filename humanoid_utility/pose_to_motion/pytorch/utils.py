import sys
import argparse
from typing import List, TypedDict
import json

from humanoid_utility import humanoid_config

sys.path.append(".")
import pandas as pd
import torch
from torch.utils.data import Dataset, DataLoader
import torch.nn as nn
import optuna
from optuna.trial import TrialState
import torch.optim as optim
from sklearn.metrics import mean_squared_error
from tqdm import tqdm
from pathlib import Path
import os
import random


MODEL_STATES_DIR = Path(__file__).parent / "output/saved_model_states"

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

def get_hyperparameters() -> HyperparameterDict:
    return HyperparameterDict({   
            # TRAIN LOOP SPECIFIC
            "N_EPOCHS": 5,
            "BATCHSIZE": 22,
            "optimizer_name": "Adam",
            "lr":7.088648070427078e-05,
            "n_layers": 3,
            # MODEL SPECIFIC
            "size_per_layer": [504, 476, 479],
            "dropout_per_layer": [0.13201141925241577, 0.20817940982863983, 0.14885002648956877],
            "in_features": humanoid_config.NUM_MARKERS * 3,
            "out_features": humanoid_config.NUM_JOINTS
        })

class MultiLabelRegressionDataset(Dataset):
    def __init__(self, csv_file, num_features):
        data = pd.read_csv(csv_file)
        self.features = torch.tensor(
            data.iloc[:, 0 : num_features ].values, dtype=torch.float32
        )  # first column is index, then features
        self.labels = torch.tensor(
            data.iloc[:, num_features  :].values, dtype=torch.float32
        )  # Last num_feat columns as labels

    def __len__(self):
        return len(self.features)

    def __getitem__(self, idx):
        return self.features[idx], self.labels[idx]




def define_torch_model(hyperparameters:HyperparameterDict):
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
    if(model_instance == "optuna"):
        return
    # If model exists already we add a random identifier to it so we never overwrite models.
    if(model_exist(model_instance)):
        random_identifier = int(random.random()*100)
        model_instance = model_instance + f"_copy_{random_identifier}"
        print("model already exists, saving a new copy")
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
            print(f"Unable to load model with name {model_instance}. Most probably the hyperparameters missmatch.")
            print(e)

    else:
        print("No model with this name found. Using new model")
    return model

def model_exist(model_instance):
    # We check if file exist.
    my_file = Path(f"{MODEL_STATES_DIR}/{model_instance}.pth")
    return my_file.exists()