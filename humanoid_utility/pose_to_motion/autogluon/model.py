import os
import numpy as np
import pandas as pd
import sys
import argparse
import json
import time

sys.path.append(".")
import humanoid_utility.humanoid_config as humanoid_config
import humanoid_utility.pre_processing.load_json_data as load_json_data
from humanoid_utility.pose_to_motion.generate_report import (
    generate_eval_report,
    generate_train_report,
)
from humanoid_utility.pose_to_motion.autogluon.utils import (
    MultilabelPredictor,
    generate_output_dirs,
)
from pathlib import Path

theta = np.pi / 2  # 90 degrees
BASE_MODEL_DIR = Path(__file__).parent
CAM_IDS = [500, 501, 502, 503]
AXES = ["x", "y", "z"]


def preprocess_normalization_single(df):
    """
    POSITION NORMALIZATION: Make all points relative to one single point (MARKER_L_HIP)
    RESULT: As the result the x,y,z for MARKER_L_HIP becomes 0
    """
    print("RAW DATA:\n", df.shape, type(df))
    print("POSITION NORMALIZATION")
    df_normalized = df.copy()
    for i in range(0, humanoid_config.NUM_MARKERS * 3, 3):
        df_normalized.iloc[:, i : i + 3] = (
            df.iloc[:, i : i + 3]
            - df.iloc[
                :,
                (3 * humanoid_config.MARKER_L_HIP) : (
                    3 * humanoid_config.MARKER_L_HIP + 3
                ),
            ].values
        )

    df_normalized.to_csv("NORM_1.csv", index=True)
    return df_normalized


def _load_pose_list_json(path: Path):
    with open(path, "r") as f:
        arr = json.load(f)
    arr = sorted(arr, key=lambda x: x["index"])
    return arr


def _to_row_with_prefix_nan(pose_list, cam_prefix: str):
    row = {}
    for j in range(humanoid_config.NUM_MARKERS):
        item = pose_list[j]
        for axis in AXES:
            row[f"{cam_prefix}_{j}_{axis}"] = float(item[axis])
    return row


def build_multicam_row_nan(sample_root: Path, sample_id: str):
    """
    Create the input dataframe for the autogluon function.
    """
    row = {}
    for cam_id in CAM_IDS:
        prefix = f"cam{cam_id}"
        p = sample_root / f"camera_{cam_id}" / "pose_data" / f"{sample_id}_pose.json"
        if p.exists():
            pose_list = _load_pose_list_json(p)
            cam_row = _to_row_with_prefix_nan(pose_list, prefix)
            row.update(cam_row)
        else:
            for j in range(humanoid_config.NUM_MARKERS):
                for axis in AXES:
                    row[f"{prefix}_{j}_{axis}"] = float("nan")
    return pd.DataFrame([row])


def autogluon_single_training(csv_filename, df, model_instance):
    """
    Train autogluon on a training dataset that corresponds to single camera
    and it saves the trained models.
    """
    # Use this for a SAMPLE OF DATASET (for testing)
    # subsample_size = 100
    # df = df.sample(n=subsample_size, random_state=0)
    # time_limit = 50

    # FULL DATASET (for actual training)
    time_limit = 500
    train_data = preprocess_normalization_single(df)
    problem_types = ["regression"] * (humanoid_config.NUM_JOINTS)
    hyperparameters = {"NN_TORCH": {}, "GBM": {}, "XGB": {}}

    path_model = BASE_MODEL_DIR / "output" / "saved_model_states" / model_instance

    print(
        "time_limit", time_limit
    )  # how many seconds to train the TabularPredictor for each label
    multi_predictor = MultilabelPredictor(
        labels=humanoid_config.movable_joint_names,
        problem_types=problem_types,
        path=path_model,
        consider_labels_correlation=False,  # disables using one label as feature for another
    )

    multi_predictor.fit(
        train_data,
        presets="best_quality",
        ag_args_fit={
            "num_gpus": 1,
            "num_bag_folds": 0,
            "num_bag_sets": 1,
        },
        time_limit=time_limit,
        hyperparameters=hyperparameters,
        dynamic_stacking=False,
        num_stack_levels=0,
        auto_stack=False,
    )
    labels = multi_predictor.labels
    print("Detected label columns:", labels)

    # Loop through each label-specific model
    for label in labels:
        print(f"\nAnalyzing label: {label}")
        # Get the specific TabularPredictor for this label
        predictor = multi_predictor.get_predictor(label)
        # Leaderboard
        leaderboard = predictor.leaderboard(silent=True)
        print(f"=== Leaderboard for {label} ===")
        print(leaderboard)
        # Identify best model
        best_model = leaderboard.iloc[0]["model"]
        print(f"Best model for '{label}': {best_model}")

    generate_train_report(
        "autogluon", model_instance, csv_filename, " ", " ", str(hyperparameters)
    )

    return None


def autogluon_multi_training(csv_file, df, model_instance):
    """
    Train autogluon on a training dataset that corresponds to multi camera
    and it saves the trained models.
    """
    # Use this for a SAMPLE OF DATASET (for testing)
    # subsample_size = 100
    # df = df.sample(n=subsample_size, random_state=0)
    # time_limit = 50

    # FULL DATASET (for actual training)
    time_limit = 500

    # Use this if a preprocessing is needed before training the model
    # train_data = preprocess_normalization(df)

    problem_types = ["regression"] * (humanoid_config.NUM_JOINTS)

    hyperparameters = {"NN_TORCH": {}, "GBM": {}, "XGB": {}}

    path_model = BASE_MODEL_DIR / "output" / "saved_model_states" / model_instance

    # How many seconds to train the TabularPredictor for each label
    print("time_limit", time_limit)
    multi_predictor = MultilabelPredictor(
        labels=humanoid_config.movable_joint_names,
        problem_types=problem_types,
        path=path_model,
        consider_labels_correlation=False,  # disables using one label as feature for another
    )
    multi_predictor.fit(
        df,
        presets="best_quality",
        ag_args_fit={
            "num_gpus": 1,
            "num_bag_folds": 0,
            "num_bag_sets": 1,
        },
        time_limit=time_limit,
        hyperparameters=hyperparameters,
        dynamic_stacking=False,
        num_stack_levels=0,
        auto_stack=False,
    )

    labels = multi_predictor.labels
    print("Detected label columns:", labels)

    # Loop through each label-specific model
    for label in labels:
        print(f"\nAnalyzing label: {label}")
        # Get the specific TabularPredictor for this label
        predictor = multi_predictor.get_predictor(label)
        # Leaderboard
        leaderboard = predictor.leaderboard(silent=True)
        print(f"=== Leaderboard for {label} ===")
        print(leaderboard)
        # Identify best model
        best_model = leaderboard.iloc[0]["model"]
        print(f"Best model for '{label}': {best_model}")
    generate_train_report(
        "autogluon", model_instance, csv_file, " ", " ", str(hyperparameters)
    )
    return None


def autogluon_evaluation(csv_filename, model_instance):
    """
    Evaluate the trained models on a evaluation datasets and save the output as a json file.
    """
    dataset_name = Path(csv_filename).stem
    dataset_training = dataset_name.replace("eval", "train")

    model_instance = BASE_MODEL_DIR / "output" / "saved_model_states" / model_instance
    test_data = pd.read_csv(csv_filename, index_col=0)
    test_data_nolab = test_data.drop(columns=humanoid_config.movable_joint_names)
    multi_predictor = MultilabelPredictor.load(model_instance)
    predictions = multi_predictor.predict(test_data_nolab)
    print("Predictions:  \n", predictions)
    evaluations = multi_predictor.evaluate(test_data)
    print(evaluations)
    print("Evaluated using metrics:", multi_predictor.eval_metrics)
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
        "num_test_samples": len(test_data),
        "overall_mse": overall_mse,
        "overall_rmse": overall_rmse,
        "overall_mae": overall_mae,
    }

    timestr = time.strftime("%Y%m%d-%H%M%S")
    output_file_result = f"{model_instance}_{dataset_training}_{timestr}_results.json"
    output_file_result = Path(__file__).parent / "output" / output_file_result
    with open(output_file_result, "w") as f:
        json.dump(eval_results, f, indent=2)
    print(f" Evaluation results saved to {output_file_result}")
    generate_eval_report(
        "autogluon",
        model_instance,
        csv_filename,
        overall_mse,
        overall_mae,
        "",
    )
    return None


def autogluon_multi_prediction(save_path, args):
    """
    Loads the trained MultilabelPredictor models for multi camera inputs and performs prediction, and saves the result to a JSON file.
    """
    # Load the trained predictor
    multi_predictor = MultilabelPredictor.load(save_path)

    # Build dataframe from sample if available
    if args.sample_root and args.sample_id:
        sample_root = Path(args.sample_root)
        df = build_multicam_row_nan(sample_root, args.sample_id)

        # Reindex dataframe to match predictor’s expected features
        try:
            expected_columns = list(multi_predictor.feature_metadata.get_features())
            df = df.reindex(columns=expected_columns, fill_value=np.nan)
        except Exception:
            pass
    else:
        raise ValueError(
            "Both 'sample_root' and 'sample_id' must be provided for prediction."
        )

    # Perform prediction
    predicted_motion = multi_predictor.predict(df)
    print("Predicted motion:\n", predicted_motion, type(predicted_motion))

    # Convert the first row of predictions to a dictionary
    data_dict = predicted_motion.iloc[0].to_dict()
    print(data_dict)

    # Save prediction result to JSON
    with open(args.motion_file, "w") as f:
        json.dump(data_dict, f, indent=2)

    return data_dict


def autogluon_single_prediction(save_path, args):
    """
    Loads the trained MultilabelPredictor models for single camera inputs and performs prediction, and saves the result to a JSON file.
    """
    p_val, p_key = load_json_data.load_pose_from_json(args.pose_file)
    df = preprocess_normalization_single(
        pd.DataFrame([p_val], columns=humanoid_config.pose_names)
    )
    multi_predictor = MultilabelPredictor.load(save_path)
    predicted_motion = multi_predictor.predict(df)
    print("Predicted motion:\n", predicted_motion, type(predicted_motion))
    data_dict = predicted_motion.iloc[0].to_dict()
    print(data_dict)
    with open(args.motion_file, "w") as f:
        json.dump(data_dict, f, indent=2)
    return data_dict


if __name__ == "__main__":
    generate_output_dirs()
    parser = argparse.ArgumentParser(description="Model control")
    parser.add_argument(
        "--mode",
        type=str,
        choices=[
            "single_train",
            "multi_train",
            "single_predict",
            "multi_predict",
            "eval",
        ],
        required=True,
        help="Mode in which to run the script: train, test, or evaluate",
    )
    parser.add_argument("--dataset_filename", type=str, help="CSV file")
    parser.add_argument("--pose_file", type=str, help="JSON file")
    parser.add_argument("--motion_file", type=str, help="JSON file")
    parser.add_argument(
        "--sample_root", type=str, help="Root that contains cameras directories"
    )
    parser.add_argument(
        "--sample_id", type=str, help="used to find the pose and motion json files"
    )
    parser.add_argument(
        "--model_instance", type=str, help="Model name to either create or load", default=""
    )
   

    args = parser.parse_args()

    if args.mode == "multi_train":
        csv_filename = args.dataset_filename
        df = pd.read_csv(csv_filename, index_col=0)
        autogluon_multi_training(csv_filename, df, args.model_instance)
    elif args.mode == "eval":
        csv_filename = args.dataset_filename
        autogluon_evaluation(csv_filename, args.model_instance)

    elif args.mode == "multi_predict":
        model_path = BASE_MODEL_DIR / "output" / "saved_model_states" / args.model_instance
        autogluon_multi_prediction(model_path, args)

    elif args.mode == "single_train":
        csv_filename = args.dataset_filename
        df = pd.read_csv(csv_filename, index_col=0)
        autogluon_single_training(csv_filename, df, args.model_instance)

    elif args.mode == "single_predict":
        model_path = BASE_MODEL_DIR / "output" / "saved_model_states" / args.model_instance
        autogluon_single_prediction(model_path, args)
