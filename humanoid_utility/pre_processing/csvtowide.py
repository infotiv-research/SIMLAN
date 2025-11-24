#!/usr/bin/env python3
"""
Convert CSV format from long to wide format for pose data.
Usage:
    python csvtowide.py <input.csv> <output_wide.csv>

This script will:
- Convert the input long-format CSV into a wide-format CSV (all cameras combined)
- Also save one wide-format CSV per camera
"""

import pandas as pd
import sys
import os


def convert_to_wide(input_csv, output_csv):
    """Convert regular CSV to wide format and save per single camera + multi camera CSVs"""
    # Read the CSV file
    df = pd.read_csv(input_csv)
    print("original_shape", df.shape)

    # Identify columns
    pose_cols = [c for c in df.columns if "_" in c and c.split("_")[0].isdigit()]
    motion_cols = [
        c for c in df.columns if c not in ["motion_id", "camera_id"] + pose_cols
    ]
    print(
        "pose_cols", len(pose_cols), pose_cols[:5] if len(pose_cols) > 5 else pose_cols
    )
    print(
        "motion_cols",
        len(motion_cols),
        motion_cols[:5] if len(motion_cols) > 5 else motion_cols,
    )

    # Ensure output directory exists
    output_dir = os.path.dirname(output_csv) or "."
    os.makedirs(output_dir, exist_ok=True)

    # Combined wide CSV for all cameras
    wide = df.pivot(index="motion_id", columns="camera_id", values=pose_cols)
    wide.columns = [f"cam{cam}_{feat}" for feat, cam in wide.columns.to_flat_index()]
    wide = wide.reset_index()

    # Merge with motion data
    motion_df = df.drop_duplicates("motion_id")[["motion_id"] + motion_cols]
    result_all = pd.merge(wide, motion_df, on="motion_id", how="left")

    # Sort and drop motion_id for output
    result_all = result_all.sort_values("motion_id").drop("motion_id", axis=1)

    # Save main combined file
    result_all.to_csv(output_csv, index=False)
    print("final_shape (all cameras):", result_all.shape)
    print(f"Saved combined wide CSV -> {output_csv}")
    print("final_shape_multi_camera", result_all.shape)
    print(
        "Sample columns of the multi_camera csv file:", result_all.columns[:10].tolist()
    )

    # Separate CSVs for each single camera
    for cam_id, cam_df in df.groupby("camera_id"):
        wide_cam = cam_df[["motion_id"] + pose_cols].copy()

        # Merge with motion-level metadata
        motion_df_cam = cam_df.drop_duplicates("motion_id")[["motion_id"] + motion_cols]
        result_cam = pd.merge(wide_cam, motion_df_cam, on="motion_id", how="left")

        # Sort and drop motion_id (same style as main output)
        result_cam = result_cam.sort_values("motion_id").drop("motion_id", axis=1)

        # Save to file
        base, ext = os.path.splitext(output_csv)
        new_base = base.replace("multi", "single")
        output_cam_csv = f"{new_base}_cam{cam_id}{ext}"
        result_cam.to_csv(output_cam_csv, index=False)
        print(f"Saved wide CSV for single camera {cam_id} -> {output_cam_csv}")
        print("final_shape_single_camera", result_cam.shape)
        print(
            "Sample columns of the single_camera csv file:",
            result_cam.columns[:10].tolist(),
        )

    print("All files generated successfully.")


def main():
    if len(sys.argv) != 3:
        print("Usage: python csvtowide.py <input.csv> <output_wide.csv>")
        sys.exit(1)

    input_csv = sys.argv[1]
    output_csv = sys.argv[2]

    if not os.path.exists(input_csv):
        print(f"Error: Input file {input_csv} not found")
        sys.exit(1)

    convert_to_wide(input_csv, output_csv)


if __name__ == "__main__":
    main()
