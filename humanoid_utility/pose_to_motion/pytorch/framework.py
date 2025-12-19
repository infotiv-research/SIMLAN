from torch.utils.data import random_split
import sys
import torch
from torch.utils.data import DataLoader
import torch.nn as nn

import torch.optim as optim
from sklearn.metrics import mean_squared_error
from pathlib import Path
import pandas as pd

sys.path.append(".")

from humanoid_utility.pose_to_motion import metrics
from humanoid_utility.pose_to_motion.generate_report import (
    generate_eval_report,
    generate_train_report,
)
from humanoid_utility.pose_to_motion.pytorch.utils import (
    MultiLabelRegressionDataset,
    get_hyperparameters,
    load_model,
    save_model,
)


# TODO Change input_dir to report_dir for clarity
class PytorchFramework:
    def __init__(self, model_instance: str, camera_ids: list[str]):
        generate_output_dirs()
        self.model_instance = model_instance
        self.camera_ids = camera_ids
        self.hyperparameters = get_hyperparameters(len(camera_ids))
        self.model = load_model(model_instance, self.hyperparameters)

    def train(self, report_dir_name, train_df: pd.DataFrame):
        train_df.fillna(train_df.mean(), inplace=True)
        # Create train and validation dataset by splitting with set ratio.
        dataset = MultiLabelRegressionDataset(train_df, self.camera_ids)
        train_val_split = 0.8
        train_size = int(len(dataset) * train_val_split)
        valid_size = len(dataset) - train_size
        train_dataset, validation_dataset = random_split(
            dataset,
            [train_size, valid_size],
            generator=torch.Generator().manual_seed(42),  # for reproducibility
        )
        # Define the train and validation data_loaders
        train_loader = DataLoader(
            train_dataset, batch_size=self.hyperparameters["BATCHSIZE"], shuffle=False
        )
        valid_loader = DataLoader(validation_dataset, batch_size=128, shuffle=False)

        # Define criterion
        criterion = nn.MSELoss()

        # Generate the optimizers.
        optimizer = getattr(optim, self.hyperparameters["optimizer_name"])(
            self.model.parameters(), lr=self.hyperparameters["lr"]
        )

        # Put model on device
        DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(DEVICE)

        # Training of the model.
        for epoch in range(self.hyperparameters["N_EPOCHS"]):
            self.model.train()
            mse_total = 0.0

            for data, target in train_loader:

                data, target = data.view(data.size(0), -1).to(DEVICE), target.to(DEVICE)

                optimizer.zero_grad()
                output = self.model(data)
                loss = criterion(output, target)
                loss.backward()
                optimizer.step()

            # Validation of the model.
            self.model.eval()
            with torch.no_grad():
                for data, target in valid_loader:

                    data, target = data.view(data.size(0), -1).to(DEVICE), target.to(
                        DEVICE
                    )
                    predictions_cpu = self.model(data).detach().cpu().numpy()
                    target_cpu = target.detach().cpu().numpy()
                    mse = mean_squared_error(predictions_cpu, target_cpu)

                    mse_total += mse
            mse_average = mse_total / len(valid_loader)

            print(
                f"Epoch {epoch+1} / {self.hyperparameters['N_EPOCHS']}, MSE: {mse_average:.8f}",
                flush=True,
            )
        # After we are done training we save the model.
        model_instance = save_model(self.model, self.model_instance)
        # We finally generate a report
        generate_train_report(
            "pytorch",
            model_instance,
            report_dir_name,
            self.camera_ids,
            mse_average,
            " ",
            str(self.hyperparameters),
        )

    def evaluate(self, report_dir_name: pd.DataFrame, eval_df):
        eval_df.fillna(eval_df.mean(), inplace=True)

        # Set device to cpu
        DEVICE = "cpu"
        ## Dataset
        test_dataset = MultiLabelRegressionDataset(eval_df, self.camera_ids)
        test_loader = DataLoader(test_dataset, batch_size=1, shuffle=False)
        targets, predictions = [], []
        ## We get the predictions
        with torch.no_grad():
            for data, target in test_loader:
                data, target = data.view(data.size(0), -1).to(DEVICE), target.to(DEVICE)
                prediction = self.model(data)
                targets.append(target)
                predictions.append(prediction)

        # We use predicts and target to evaluate the performance
        mse_avg, mae_avg = metrics.eval(predictions, targets)

        # We finish by generating a report of the run
        generate_eval_report(
            "pytorch",
            self.model_instance,
            report_dir_name,
            self.camera_ids,
            mse_avg,
            mae_avg,
            self.hyperparameters,
        )

    def predict(self, poses_df):
        print("Pytorch predict")
        poses_df = poses_df.drop("motion_id", axis=1, errors="ignore")
        predicted_motions = []
        for i in poses_df.index:
            pose_landmarks_numpy = poses_df.to_numpy()[i]
            predicted_motion = (
                self.model(torch.Tensor(pose_landmarks_numpy))
                .cpu()
                .detach()
                .numpy()
                .tolist()
            )
            predicted_motions.append(predicted_motion)
        return predicted_motions


def generate_output_dirs():

    # Base output paths
    output_folder = Path(__file__).parent / "output"
    reports_folder = output_folder / "reports"
    model_states_folder = output_folder / "saved_model_states"

    # Create directories (including parents)
    output_folder.mkdir(parents=True, exist_ok=True)
    reports_folder.mkdir(parents=True, exist_ok=True)
    model_states_folder.mkdir(parents=True, exist_ok=True)

    # Verify existence
    assert (
        model_states_folder.is_dir()
    ), "Model states dir not found or could not be generated"
    assert reports_folder.is_dir(), "Reports dir not found or could not be generated"
