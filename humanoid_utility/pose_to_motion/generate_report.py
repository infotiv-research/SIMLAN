#!/usr/bin/env python3

import pandas as pd
import datetime
from pathlib import Path
import os


def generate_train_report(
    model_type="No data",
    model_instance="No data",
    train_dataset="No data",
    cameras_used="No data",
    val_mse=0.0,
    val_mae=0.0,
    hyperparameters="No data",
):

    assert model_type in [
        "pytorch",
        "autogluon",
        "No data",
    ], "The model type should be in lowercase, and either 'pytorch' or 'autogluon'."

    report_file = Path(__file__).parent / model_type / "output/reports/train_report.csv"

    columns = [
        "MODEL_INSTANCE",
        "DATE",
        "MODEL_TYPE",
        "TRAIN_DATASET",
        "CAMERAS_USED",
        "VAL_MSE",
        "VAL_MAE",
        "HYPERPARAMETERS",
    ]
    entry_values = [
        model_instance,
        datetime.datetime.now(),
        model_type,
        train_dataset,
        cameras_used,
        val_mse,
        val_mae,
        hyperparameters,
    ]
    new_row = pd.DataFrame([entry_values], columns=columns)

    if not report_file.exists():
        print(f"Report file not found. Generating new one at: {report_file}")
        new_row.to_csv(report_file, index=False)
    else:
        report_df = pd.read_csv(report_file)
        report_df = pd.concat([report_df, new_row], ignore_index=True)
        report_df.to_csv(report_file, index=False)


def generate_eval_report(
    model_type="No data",
    model_instance="No data",
    eval_dataset="No data",
    cameras_used="No data",
    avg_mse=0.0,
    avg_mae=0.0,
    hyperparameters="No data",
):

    assert model_type in [
        "pytorch",
        "autogluon",
        "No data",
    ], "The model type should be in lowercase, and either 'pytorch' or 'autogluon'."

    columns = [
        "MODEL_INSTANCE",
        "DATE",
        "MODEL_TYPE",
        "EVAL_DATASET",
        "CAMERAS_USED",
        "AVG_MSE",
        "AVG_MAE",
        "HYPERPARAMETERS",
    ]
    entry_values = [
        model_instance,
        datetime.datetime.now(),
        model_type,
        eval_dataset,
        cameras_used,
        avg_mse,
        avg_mae,
        hyperparameters,
    ]
    new_row = pd.DataFrame([entry_values], columns=columns)

    report_file = Path(__file__).parent / model_type / "output/reports/eval_report.csv"
    if not report_file.exists():
        print(f"Report file not found. Generating new one at: {report_file}")

        new_row.to_csv(report_file, index=False)
    else:
        report_df = pd.read_csv(report_file)
        report_df = pd.concat([report_df, new_row], ignore_index=True)
        report_df.to_csv(report_file, index=False)
