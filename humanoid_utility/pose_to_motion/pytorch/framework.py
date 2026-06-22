from torch.utils.data import random_split, DataLoader
import torch
import torch.nn as nn
import torch.optim as optim
from pathlib import Path
import pandas as pd
from humanoid_utility.pose_to_motion.generate_report import (
    generate_eval_report,
    generate_train_report,
)
from humanoid_utility.pose_to_motion.pytorch.utils import (
    MultiLabelRegressionDataset,
    get_hyperparameters,
    load_model,
    save_model,
    load_normalization_stats,
    save_normalization_stats,
)


class PytorchFramework:
    def __init__(self, model_instance: str, camera_ids: list[str]):
        generate_output_dirs()
        self.model_instance = model_instance
        self.camera_ids = camera_ids
        self.hyperparameters = get_hyperparameters(len(camera_ids))
        self.model = load_model(model_instance, self.hyperparameters)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)

        # Normalization statistics - try to load from disk
        stats = load_normalization_stats(model_instance)
        if stats is not None:
            self.feature_mean = stats["feature_mean"]
            self.feature_std = stats["feature_std"]
            self.label_mean = stats["label_mean"]
            self.label_std = stats["label_std"]
        else:
            self.feature_mean = None
            self.feature_std = None
            self.label_mean = None
            self.label_std = None

        # Cache normalized statistics on device for efficiency (for loaded models)
        self._update_device_cache()

    def _update_device_cache(self):
        """Cache normalization statistics on device to avoid repeated transfers."""
        if self.feature_mean is not None:
            self.feature_mean_device = self.feature_mean.to(self.device)
            self.feature_std_device = self.feature_std.to(self.device)
        else:
            self.feature_mean_device = None
            self.feature_std_device = None

        if self.label_mean is not None:
            self.label_mean_device = self.label_mean.to(self.device)
            self.label_std_device = self.label_std.to(self.device)
        else:
            self.label_mean_device = None
            self.label_std_device = None

    def _normalize_per_camera(self, data: torch.Tensor) -> torch.Tensor:
        """Normalize data per-camera using per-camera statistics."""
        B, F = data.shape
        N_cameras = len(self.camera_ids)
        features_per_camera = F // N_cameras

        # Reshape to (B, N_cameras, features_per_camera)
        x = data.view(B, N_cameras, features_per_camera)

        # Normalize per-camera
        if self.feature_mean_device is not None and self.feature_std_device is not None:
            x = (x - self.feature_mean_device) / self.feature_std_device

        return x.view(B, F)

    def root_center_pose(self, data: torch.Tensor) -> torch.Tensor:
        B, F = data.shape
        J, D = 33, 3
        N_cameras = F // (J * D)

        x = data.view(B, J, D, N_cameras).permute(0, 3, 1, 2)

        pelvis = x[:, :, [23, 24], :].mean(dim=2, keepdim=True)

        x = x - pelvis

        return x.permute(0, 2, 3, 1).reshape(B, F)

    def add_realistic_pose_noise(self, data: torch.Tensor) -> torch.Tensor:
        """
        Realistic pose augmentation in normalized space.
        Pelvis (midpoint of hips) is not augmented to preserve root stability.
        Handles multi-camera data by processing each camera separately.
        """

        B, F = data.shape
        J, D = 33, 3

        # Calculate number of cameras from feature dimension
        N_cameras = F // (J * D)

        x = data.view(B, J, D, N_cameras).permute(0, 3, 1, 2)

        pelvis_indices = [23, 24]

        # Small Gaussian jitter
        jitter_std = 0.03
        # Apply noise to all joints except pelvis
        mask_jitter = torch.ones_like(x, dtype=torch.bool)
        mask_jitter[:, :, pelvis_indices, :] = 0
        noise = torch.randn_like(x) * jitter_std
        x = torch.where(mask_jitter, x + noise, x)

        # Joint dropout
        joint_drop_prob = 0.03
        mask_dropout = torch.rand(B, N_cameras, J, 1, device=x.device) < joint_drop_prob
        # exclude pelvis
        mask_dropout[:, :, pelvis_indices, :] = 0
        dropout_noise = torch.randn_like(x) * 0.05
        x = torch.where(mask_dropout, dropout_noise, x)

        # Limb dropout
        limb_drop_prob = 0.01
        if limb_drop_prob > 0:
            limbs = [
                [25, 27, 29],  # left leg: knee, ankle, heel
                [26, 28, 30],  # right leg: knee, ankle, heel
                [11, 13, 15],  # left arm: shoulder, elbow, wrist
                [12, 14, 16],  # right arm: shoulder, elbow, wrist
            ]

            for b in range(B):
                for c in range(N_cameras):
                    if torch.rand((), device=x.device) < limb_drop_prob:
                        limb = limbs[torch.randint(len(limbs), ()).item()]
                        for j in limb:
                            if j < J and j not in pelvis_indices:  # exclude pelvis
                                x[b, c, j] = torch.randn(D, device=x.device) * 0.05

        return x.permute(0, 2, 3, 1).reshape(B, F)

    def train(self, report_dir_name: str, train_df: pd.DataFrame):
        # Fill missing values with median to ensure all samples are usable for training.
        train_df.fillna(train_df.median(), inplace=True)

        dataset = MultiLabelRegressionDataset(train_df, self.camera_ids)

        train_size = int(0.8 * len(dataset))
        valid_size = len(dataset) - train_size

        train_dataset, valid_dataset = random_split(
            dataset,
            [train_size, valid_size],
            generator=torch.Generator().manual_seed(42),
        )

        train_indices = train_dataset.indices
        train_features = dataset.features[train_indices]

        B, F = train_features.shape

        # root-center at pelvis
        x_centered = self.root_center_pose(train_features)
        x_centered_flat = x_centered.view(B, F)

        # compute per-camera normalization statistics on centered TRAINING data only
        N_cameras = len(self.camera_ids)
        features_per_camera = F // N_cameras
        x_by_camera = x_centered_flat.view(B, N_cameras, features_per_camera)
        self.feature_mean = x_by_camera.mean(dim=0)
        self.feature_std = x_by_camera.std(dim=0) + 1e-6

        train_labels = dataset.labels[train_indices]
        self.label_mean = train_labels.mean(dim=0)
        self.label_std = train_labels.std(dim=0) + 1e-6  # safe division

        # Cache normalized statistics on device for efficiency (avoid repeated transfers)
        self._update_device_cache()

        train_loader = DataLoader(
            train_dataset,
            batch_size=self.hyperparameters["BATCHSIZE"],
            shuffle=True,
        )

        valid_loader = DataLoader(
            valid_dataset,
            batch_size=256,
            shuffle=False,
        )
        # SmoothL1Loss(beta=0.1) is appropriate for multi-joint regression: smooth for small errors, robust to outliers.
        criterion = nn.SmoothL1Loss(beta=0.1)
        # weight_decay (L2) to prevent overfitting,
        optimizer = getattr(optim, self.hyperparameters["optimizer_name"])(
            self.model.parameters(),
            lr=self.hyperparameters["lr"],
            weight_decay=self.hyperparameters["weight_decay"],
        )

        # CosineAnnealingWarmRestarts for smooth convergence without overfitting runaway
        scheduler = optim.lr_scheduler.CosineAnnealingWarmRestarts(
            optimizer,
            T_0=10,  # number of epochs before first restart
            T_mult=2,  # multiply T_0 by 2 after each restart
            eta_min=self.hyperparameters.get("lr_min", 1e-6),
        )

        best_val_mse_denorm = float("inf")  # Track denormalized MSE for early stopping
        patience = 15
        patience_counter = 0

        for epoch in range(self.hyperparameters["N_EPOCHS"]):
            self.model.train()
            train_loss_total = 0.0
            train_samples = 0

            for data, target in train_loader:
                data = data.view(data.size(0), -1).to(self.device)
                target = target.to(self.device)

                # Preprocess: root-center and normalize per-camera
                data = self.root_center_pose(data)
                data = self._normalize_per_camera(data)

                # Augmentation: add realistic noise
                data = self.add_realistic_pose_noise(data)

                # Normalize target
                if self.label_mean_device is not None:
                    target_norm = (
                        target - self.label_mean_device
                    ) / self.label_std_device
                else:
                    target_norm = target

                optimizer.zero_grad()
                output = self.model(data)
                loss = criterion(output, target_norm)
                loss.backward()
                # Helps prevent exploding gradients especially for pose/motion networks.
                torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
                optimizer.step()

                train_loss_total += loss.item() * data.size(0)
                train_samples += data.size(0)

            train_loss_avg = train_loss_total / train_samples

            self.model.eval()
            val_losses_per_joint = torch.zeros(target.size(1), device=self.device)
            val_losses_per_joint_denorm = None  # Only compute if stats are available

            with torch.no_grad():
                for data, target in valid_loader:
                    data = data.view(data.size(0), -1).to(self.device)
                    target = target.to(self.device)

                    # Preprocess: root-center and normalize per-camera (no augmentation on validation)
                    data = self.root_center_pose(data)
                    if self.feature_mean_device is not None:
                        data = self._normalize_per_camera(data)
                    # Normalize target for model output comparison
                    if (
                        self.label_mean_device is not None
                        and self.label_std_device is not None
                    ):
                        target_norm = (target - self.label_mean_device) / (
                            self.label_std_device + 1e-6
                        )
                    else:
                        target_norm = target

                    output = self.model(data)

                    val_losses_per_joint += ((output - target_norm) ** 2).sum(dim=0)

                    # Also compute denormalized loss for early stopping (only if stats available)
                    if self.label_mean is not None and self.label_std is not None:
                        if val_losses_per_joint_denorm is None:
                            val_losses_per_joint_denorm = torch.zeros(
                                target.size(1), device=self.device
                            )
                        output_denorm = (
                            output * self.label_std_device + self.label_mean_device
                        )
                        val_losses_per_joint_denorm += (
                            (output_denorm - target) ** 2
                        ).sum(dim=0)

            val_mse_per_joint = val_losses_per_joint / len(valid_loader.dataset)
            val_mse_avg = val_mse_per_joint.mean().item()

            # Compute denormalized MSE only if stats are available
            if val_losses_per_joint_denorm is not None:
                val_mse_per_joint_denorm = val_losses_per_joint_denorm / len(
                    valid_loader.dataset
                )
                val_mse_avg_denorm = val_mse_per_joint_denorm.mean().item()
            else:
                val_mse_avg_denorm = (
                    val_mse_avg  # Fallback to normalized if stats not available
                )

            # Step the cosine scheduler (per epoch)
            scheduler.step()
            current_lr = optimizer.param_groups[0]["lr"]
            print(
                f"Epoch {epoch+1} | Train Loss: {train_loss_avg:.8f} | Val MSE (norm): {val_mse_avg:.8f} | Val MSE (denorm): {val_mse_avg_denorm:.8f} | LR: {current_lr:.2e}"
            )

            # Save best model based on denormalized validation MSE
            if val_losses_per_joint_denorm is not None:
                # Use denormalized metric if available
                if val_mse_avg_denorm < best_val_mse_denorm:
                    best_val_mse_denorm = val_mse_avg_denorm
                    patience_counter = 0
                    save_model(self.model, self.model_instance)
                    # Also save normalization statistics
                    save_normalization_stats(
                        self.feature_mean,
                        self.feature_std,
                        self.label_mean,
                        self.label_std,
                        self.model_instance,
                    )
                else:
                    patience_counter += 1

            if patience_counter >= patience:
                print("Early stopping triggered.")
                break

        generate_train_report(
            "pytorch",
            self.model_instance,
            report_dir_name,
            self.camera_ids,
            best_val_mse_denorm,
            " ",
            str(self.hyperparameters),
        )

    def evaluate(self, report_dir_name: str, eval_df: pd.DataFrame):

        eval_df.fillna(eval_df.median(), inplace=True)

        # Create evaluation dataset (normalization is handled in the framework)
        dataset = MultiLabelRegressionDataset(eval_df, self.camera_ids)
        loader = DataLoader(dataset, batch_size=256, shuffle=False)

        self.model.eval()

        predictions_all = []
        targets_all = []

        with torch.no_grad():
            for data, target in loader:
                data = data.view(data.size(0), -1).to(self.device)

                # Preprocess: root-center and normalize per-camera
                data = self.root_center_pose(data)
                if self.feature_mean_device is not None:
                    data = self._normalize_per_camera(data)
                else:
                    print("Warning: Normalization statistics not available.")

                target = target.to(self.device)

                # Normalize target
                if (
                    self.label_mean_device is not None
                    and self.label_std_device is not None
                ):
                    target_norm = (
                        target - self.label_mean_device
                    ) / self.label_std_device
                else:
                    target_norm = target

                output = self.model(data)

                predictions_all.append(output.cpu())
                targets_all.append(target_norm.cpu())

        predictions_norm = torch.cat(predictions_all, dim=0)
        targets_norm = torch.cat(targets_all, dim=0)

        # Denormalize predictions and targets for evaluation
        if self.label_mean is not None and self.label_std is not None:
            predictions_denorm = predictions_norm * self.label_std + self.label_mean
            targets_denorm = targets_norm * self.label_std + self.label_mean
        else:
            predictions_denorm = predictions_norm
            targets_denorm = targets_norm

        mse_avg = torch.mean((predictions_denorm - targets_denorm) ** 2).item()
        mae_avg = torch.mean(torch.abs(predictions_denorm - targets_denorm)).item()

        # Max joint error across all samples
        if self.label_mean is not None and self.label_std is not None:
            max_joint_error = torch.max(
                torch.abs(predictions_denorm - targets_denorm)
            ).item()
            print(f"Max joint error: {max_joint_error:.6f}")
            per_joint_max_error = torch.max(
                torch.abs(predictions_denorm - targets_denorm), dim=0
            ).values
            print("Max error per joint:", per_joint_max_error.tolist())
        else:
            print(
                "Warning: Cannot compute denormalized errors (normalization stats unavailable)"
            )

        generate_eval_report(
            "pytorch",
            self.model_instance,
            report_dir_name,
            self.camera_ids,
            mse_avg,
            mae_avg,
            self.hyperparameters,
        )

    def predict(self, poses_df: pd.DataFrame):

        poses_df = poses_df.drop("motion_id", axis=1, errors="ignore")

        self.model.eval()

        input_tensor = torch.tensor(
            poses_df.to_numpy(),
            dtype=torch.float32,
        ).to(self.device)

        # Preprocess: root-center and normalize per-camera
        input_tensor = self.root_center_pose(input_tensor)
        if self.feature_mean_device is not None:
            input_tensor = self._normalize_per_camera(input_tensor)
        else:
            print("Warning: Normalization statistics not available. Using raw input.")

        with torch.no_grad():
            predictions_norm = self.model(input_tensor)

        # Denormalize predictions
        if self.label_mean_device is not None and self.label_std_device is not None:
            predictions = (
                predictions_norm * self.label_std_device + self.label_mean_device
            )
        else:
            predictions = predictions_norm

        return predictions.cpu().numpy().tolist()


def generate_output_dirs():

    output_folder = Path(__file__).parent / "output"
    reports_folder = output_folder / "reports"
    model_states_folder = output_folder / "saved_model_states"

    output_folder.mkdir(parents=True, exist_ok=True)
    reports_folder.mkdir(parents=True, exist_ok=True)
    model_states_folder.mkdir(parents=True, exist_ok=True)

    assert model_states_folder.is_dir()
    assert reports_folder.is_dir()
