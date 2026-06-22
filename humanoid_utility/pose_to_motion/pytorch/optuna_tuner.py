import argparse
import json
import math
from pathlib import Path
import sys
import torch
from torch.utils.data import Dataset

repo_root = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(repo_root))
from humanoid_utility import humanoid_config

from humanoid_utility.pose_to_motion.pytorch.utils import define_torch_model
import optuna
from optuna.trial import TrialState
import pandas as pd
import torch.nn as nn
import torch.optim as optim


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", required=True, help="Path to tabular dataset CSV")
    p.add_argument("--n-trials", type=int, default=500)
    p.add_argument("--n-jobs", type=int, default=1)
    p.add_argument(
        "--study-name",
        type=str,
        default="pytorch_optuna_study_overfitting_test_cam500_noem_aug",
    )
    return p.parse_args()


def make_hparam_dict(trial, in_features, out_features):
    # Sample common hyperparameters. Ranges are conservative and sensible.
    n_layers = trial.suggest_int("n_layers", 1, 4)
    size_per_layer = []
    dropout_per_layer = []
    for i in range(n_layers):
        # log scale for hidden size
        size = trial.suggest_int(f"hidden_{i}", 16, 512, log=True)
        size_per_layer.append(int(size))
        dropout_per_layer.append(trial.suggest_float(f"dropout_{i}", 0.0, 0.5))

    hparams = {
        "N_EPOCHS": trial.suggest_int("N_EPOCHS", 10, 80),
        "BATCHSIZE": trial.suggest_categorical("BATCHSIZE", [32, 64, 128]),
        "optimizer_name": trial.suggest_categorical(
            "optimizer_name", ["Adam", "AdamW", "SGD"]
        ),
        "lr": trial.suggest_float("lr", 1e-5, 1e-3, log=True),
        "weight_decay": trial.suggest_float("weight_decay", 1e-6, 1e-2, log=True),
        "n_layers": n_layers,
        "size_per_layer": size_per_layer,
        "dropout_per_layer": dropout_per_layer,
        "in_features": in_features,
        "out_features": out_features,
        "activation": trial.suggest_categorical(
            "activation", ["relu", "leaky_relu", "elu"]
        ),
        "use_batchnorm": trial.suggest_categorical("use_batchnorm", [False, True]),
    }
    return hparams


class MultiLabelRegressionDataset(Dataset):
    def __init__(
        self,
        dataset_df,
        marker_cols=None,
        normalize=True,
        feature_stats=None,
        label_stats=None,
    ):
        # Select feature columns
        if marker_cols is None:
            marker_cols = [
                col for col in dataset_df.columns if col.startswith("marker")
            ]
        features_raw = torch.tensor(dataset_df[marker_cols].values, dtype=torch.float32)

        # Select label columns
        labels_raw = torch.tensor(
            dataset_df[humanoid_config.movable_joint_names].values, dtype=torch.float32
        )

        # Normalization
        if normalize:
            if feature_stats is None:  # Compute stats from training data
                self.feature_mean = features_raw.mean(dim=0)
                self.feature_std = features_raw.std(dim=0)
                # Avoid division by zero
                self.feature_std[self.feature_std == 0] = 1.0
            else:  # Use provided stats from training data
                self.feature_mean = feature_stats["mean"]
                self.feature_std = feature_stats["std"]

            if label_stats is None:  # Compute stats from training data
                self.label_mean = labels_raw.mean(dim=0)
                self.label_std = labels_raw.std(dim=0)
                # Avoid division by zero
                self.label_std[self.label_std == 0] = 1.0
            else:  # Use provided stats from training data
                self.label_mean = label_stats["mean"]
                self.label_std = label_stats["std"]

            # Normalize data
            self.features = (features_raw - self.feature_mean) / self.feature_std
            self.labels = (labels_raw - self.label_mean) / self.label_std
        else:
            self.features = features_raw
            self.labels = labels_raw
            self.feature_mean = None
            self.feature_std = None
            self.label_mean = None
            self.label_std = None

    def __len__(self):
        return len(self.features)

    def __getitem__(self, idx):
        return self.features[idx], self.labels[idx]

    def get_normalization_stats(self):
        """Return normalization statistics for features and labels."""
        return {
            "feature_mean": self.feature_mean,
            "feature_std": self.feature_std,
            "label_mean": self.label_mean,
            "label_std": self.label_std,
        }


def objective(trial, df, device):
    # Create full dataset with normalization (statistics computed from full data)
    dataset = MultiLabelRegressionDataset(df, normalize=True)
    n = len(dataset)
    train_size = int(n * 0.8)
    valid_size = n - train_size
    generator = torch.Generator().manual_seed(42)
    train_dataset, valid_dataset = torch.utils.data.random_split(
        dataset, [train_size, valid_size], generator=generator
    )

    # Get normalization statistics for denormalization
    norm_stats = dataset.get_normalization_stats()
    label_mean = norm_stats["label_mean"]
    label_std = norm_stats["label_std"]

    in_features = dataset.features.shape[1]
    out_features = dataset.labels.shape[1]

    hparams = make_hparam_dict(trial, in_features, out_features)

    model = define_torch_model(hparams).to(device)

    optimizer_cls = (
        getattr(optim, hparams["optimizer_name"])
        if hasattr(optim, hparams["optimizer_name"])
        else optim.Adam
    )
    optimizer = optimizer_cls(
        model.parameters(),
        lr=hparams["lr"],
        weight_decay=hparams["weight_decay"],
    )
    criterion = nn.SmoothL1Loss(beta=0.1)

    train_loader = torch.utils.data.DataLoader(
        train_dataset, batch_size=hparams["BATCHSIZE"], shuffle=True
    )
    valid_loader = torch.utils.data.DataLoader(
        valid_dataset, batch_size=128, shuffle=False
    )

    # Training loop with pruning
    best_val = math.inf

    for epoch in range(hparams["N_EPOCHS"]):
        model.train()

        for xb, yb in train_loader:
            xb = xb.view(xb.size(0), -1).to(device)
            yb = yb.to(device)

            optimizer.zero_grad()
            pred = model(xb)
            loss = criterion(pred, yb)
            loss.backward()

            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)

            optimizer.step()

        # Validation
        model.eval()
        total_loss = 0.0
        total_samples = 0

        with torch.no_grad():
            for xb, yb in valid_loader:
                xb = xb.view(xb.size(0), -1).to(device)
                yb = yb.to(device)

                pred = model(xb)

                # Denormalize predictions and targets for proper MSE calculation
                pred_denorm = pred * label_std.to(device) + label_mean.to(device)
                yb_denorm = yb * label_std.to(device) + label_mean.to(device)

                batch_size = yb.size(0)
                mse = torch.mean((pred_denorm - yb_denorm) ** 2).item()

                total_loss += mse * batch_size
                total_samples += batch_size

        val_mse = total_loss / total_samples

        trial.report(val_mse, epoch)

        if trial.should_prune():
            raise optuna.exceptions.TrialPruned()

        if val_mse < best_val:
            best_val = val_mse

    return best_val


def main():
    args = parse_args()
    df = pd.read_csv(args.csv)
    print(df.head())
    device = "cuda" if torch.cuda.is_available() else "cpu"
    study_save_location = Path(__file__).parent / "output/optuna_studies"
    out_dir = Path(study_save_location)
    out_dir.mkdir(parents=True, exist_ok=True)
    db_path = out_dir / f"{args.study_name}.db"
    storage_uri = f"sqlite:///{db_path}"

    study = optuna.create_study(
        study_name=args.study_name,
        storage=storage_uri,
        load_if_exists=True,
        direction="minimize",
        pruner=optuna.pruners.SuccessiveHalvingPruner(),
    )

    # Wrap objective to pass fixed parameters
    func = lambda trial: objective(trial, df, device)
    study.optimize(func, n_trials=args.n_trials, n_jobs=args.n_jobs)

    pruned_trials = study.get_trials(deepcopy=False, states=[TrialState.PRUNED])
    complete_trials = study.get_trials(deepcopy=False, states=[TrialState.COMPLETE])

    print("Study statistics: ")
    print("  Number of finished trials: ", len(study.trials))
    print("  Number of pruned trials: ", len(pruned_trials))
    print("  Number of complete trials: ", len(complete_trials))

    best = study.best_trial
    out_path = out_dir / f"{args.study_name}_best.json"
    with out_path.open("w") as fh:
        json.dump({"value": best.value, "params": best.params}, fh, indent=2)

    print("Study finished. Best value:", best.value)
    print("Best params:")
    print(best.params)


if __name__ == "__main__":
    main()
