import numpy as np
import sys
import json
import time
from pathlib import Path

sys.path.append(".")
import humanoid_utility.humanoid_config as humanoid_config

from humanoid_utility.pose_to_motion.generate_report import (
    generate_eval_report,
    generate_train_report,
)
from humanoid_utility.pose_to_motion.autogluon.utils import (
    MultilabelPredictor,
    generate_output_dirs,
)


class AutogluonFramework:
    def __init__(self, model_instance: str, camera_ids: list[str]):
        generate_output_dirs()

        problem_types = ["regression"] * (humanoid_config.NUM_JOINTS)
        self.hyperparameters = {"NN_TORCH": {}, "GBM": {}, "XGB": {}}

        model_path = (
            Path(__file__).parent / "output" / "saved_model_states" / model_instance
        )

        if Path.is_dir(model_path):
            self.model = MultilabelPredictor.load(model_path)
        else:
            self.model = MultilabelPredictor(
                labels=humanoid_config.movable_joint_names,
                problem_types=problem_types,
                path=model_path,
                consider_labels_correlation=False,  # disables using one label as feature for another
            )

        self.model_instance = model_instance
        self.camera_ids = camera_ids

    def train(self, report_dir_name, train_df):

        time_limit = 500

        # How many seconds to train the TabularPredictor for each label
        print("time_limit", time_limit)
        self.model.fit(
            train_df,
            presets="best_quality",
            ag_args_fit={
                "num_gpus": 1,
                "num_bag_folds": 0,
                "num_bag_sets": 1,
            },
            time_limit=time_limit,
            hyperparameters=self.hyperparameters,
            dynamic_stacking=False,
            num_stack_levels=0,
            auto_stack=False,
        )

        labels = self.model.labels
        print("Detected label columns:", labels)

        # Loop through each label-specific model
        for label in labels:
            print(f"\nAnalyzing label: {label}")
            # Get the specific TabularPredictor for this label
            predictor = self.model.get_predictor(label)
            # Leaderboard
            leaderboard = predictor.leaderboard(silent=True)
            print(f"=== Leaderboard for {label} ===")
            print(leaderboard)
            # Identify best model
            best_model = leaderboard.iloc[0]["model"]
            print(f"Best model for '{label}': {best_model}")

        generate_train_report(
            "autogluon",
            self.model_instance,
            report_dir_name,
            self.camera_ids,
            " ",
            " ",
            str(self.hyperparameters),
        )
        return None

    def evaluate(self, report_dir_name, dataset_df):

        dataset_df.drop(columns=humanoid_config.movable_joint_names)

        evaluations = self.model.evaluate(dataset_df)
        print(evaluations)
        print("Evaluated using metrics:", self.model.eval_metrics)

        mse_values = [
            abs(evaluations[joint]["mean_squared_error"])
            for joint in humanoid_config.movable_joint_names
        ]
        mae_values = [
            -evaluations[joint]["mean_absolute_error"]
            for joint in humanoid_config.movable_joint_names
        ]
        overall_mse = np.mean(mse_values)
        overall_mae = np.mean(mae_values)
        overall_rmse = np.sqrt(overall_mse)
        eval_results = {
            "evaluations": evaluations,
            "num_test_samples": len(dataset_df),
            "overall_mse": overall_mse,
            "overall_rmse": overall_rmse,
            "overall_mae": overall_mae,
        }

        timestr = time.strftime("%Y%m%d-%H%M%S")
        output_file_result = (
            f"{self.model_instance}_{Path(report_dir_name).stem}_{timestr}_results.json"
        )
        output_file_result = Path(__file__).parent / "output" / output_file_result
        with open(output_file_result, "w") as f:
            json.dump(eval_results, f, indent=2)
        print(f" Evaluation results saved to {output_file_result}")
        generate_eval_report(
            "autogluon",
            self.model_instance,
            report_dir_name,
            self.camera_ids,
            overall_mse,
            overall_mae,
            "",
        )
        return None

    def predict(self, poses_df):
        predicted_motions = self.model.predict(poses_df)
        return predicted_motions.values.tolist()
