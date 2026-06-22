#!/usr/bin/env python3
"""
Script to add odd_detections list to combined JSON files.
Merges odd detection data from multiple JSON files in a folder.
Only includes entries where pred == true, organized with 'cam' field.
"""

import json
import argparse
import os
import glob
from pathlib import Path
from typing import Dict, List, Any


def load_json_file(filepath: str) -> dict:
    """Load and return JSON data from file."""
    try:
        with open(filepath, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: File {filepath} not found.")
        return {}
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON in file {filepath}.")
        return {}


def load_odd_detection_files(folder_path: str) -> Dict[str, Dict]:
    """Load all odd detection JSON files from a folder and merge them."""
    odd_data = {}

    # Find all JSON files in the folder
    json_files = glob.glob(os.path.join(folder_path, "*.json"))

    if not json_files:
        print(f"Warning: No JSON files found in {folder_path}")
        return odd_data

    print(f"Loading {len(json_files)} odd detection files from {folder_path}")

    for json_file in json_files:
        file_data = load_json_file(json_file)
        filename = os.path.splitext(os.path.basename(json_file))[
            0
        ]  # Get filename without extension

        if file_data:
            print(
                f"  Loaded {len(file_data)} entries from {os.path.basename(json_file)}"
            )
            # Add filename info to each timestamp entry
            for timestamp, data in file_data.items():
                if timestamp not in odd_data:
                    odd_data[timestamp] = {}
                odd_data[timestamp][filename] = data

    print(f"Total odd detection entries loaded: {len(odd_data)}")
    return odd_data


def add_odd_detections(
    combined_data: List[dict], odd_data: Dict[str, Dict]
) -> List[dict]:
    """Add odd_detections list to combined data based on odd detection results.
    Format: [{cam: filename, ood_score: value, pred: true}] - only for pred==true entries.
    """

    modified_count = 0
    total_odd_entries = 0

    for entry in combined_data:
        timestamp = entry.get("time_stamp")
        if timestamp is None:
            continue

        # Convert timestamp to string for lookup
        timestamp_str = str(timestamp)

        # Initialize odd_detections as empty list
        entry["odd_detections"] = []

        # Check if this timestamp has odd detection data
        if timestamp_str in odd_data:
            files_data = odd_data[timestamp_str]

            # Add entries for each file that has pred == true
            for filename, file_info in files_data.items():
                pred = file_info.get("pred", False)
                ood_score = file_info.get("ood_score", 0.0)

                # Only add if pred is true
                if pred:
                    odd_entry = {"cam": filename, "ood_score": ood_score, "pred": pred}
                    entry["odd_detections"].append(odd_entry)
                    total_odd_entries += 1

            if len(entry["odd_detections"]) > 0:
                modified_count += 1

    print(f"Added odd detections to {modified_count} timestamps")
    print(f"Total odd detection entries created: {total_odd_entries}")

    return combined_data


def process_combined_file(
    combined_file_path: str, odd_folder_path: str, output_file: str = None
):
    """Process combined file and add odd detections."""

    print(f"Loading combined file: {combined_file_path}")
    combined_data = load_json_file(combined_file_path)

    if not combined_data:
        print("Error: Could not load combined file or file is empty.")
        return

    if isinstance(combined_data, dict):
        print("Error: Combined file should contain a list of timestamp entries.")
        return

    print(f"Loaded {len(combined_data)} entries from combined file")

    # Load odd detection data
    odd_data = load_odd_detection_files(odd_folder_path)

    if not odd_data:
        print(
            "Warning: No odd detection data loaded. Continuing with empty odd_detections lists."
        )

    # Add odd detections to combined data
    enhanced_data = add_odd_detections(combined_data, odd_data)

    # Determine output file name
    if output_file is None:
        base_name, ext = os.path.splitext(combined_file_path)
        output_file = f"{base_name}_with_odd{ext}"

    # Save enhanced data
    try:
        with open(output_file, "w") as f:
            json.dump(enhanced_data, f, indent=2)
        print(f"Enhanced data saved to: {output_file}")

        # Show statistics
        timestamps_with_odd = sum(
            1 for entry in enhanced_data if len(entry.get("odd_detections", [])) > 0
        )
        print(
            f"Timestamps with odd detections: {timestamps_with_odd}/{len(enhanced_data)}"
        )

        # Count total files with pred==true across all timestamps
        total_files_with_pred = sum(
            len(entry.get("odd_detections", [])) for entry in enhanced_data
        )
        print(f"Total file entries with pred=true: {total_files_with_pred}")

    except Exception as e:
        print(f"Error saving enhanced file: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Add odd detections to combined JSON file"
    )
    parser.add_argument("combined_file", help="Path to combined JSON file")
    parser.add_argument(
        "odd_folder",
        nargs="?",
        default="odd_scores",
        help="Path to folder containing odd detection JSON files (default: odd_scores)",
    )
    parser.add_argument(
        "-o", "--output", help="Output file path (default: input_with_odd.json)"
    )

    args = parser.parse_args()

    # Validate inputs
    if not os.path.isfile(args.combined_file):
        print(f"Error: Combined file '{args.combined_file}' not found.")
        return

    if not os.path.isdir(args.odd_folder):
        print(f"Error: Odd detection folder '{args.odd_folder}' not found.")
        return

    process_combined_file(args.combined_file, args.odd_folder, args.output)


if __name__ == "__main__":
    # Example usage when run directly
    import sys

    if len(sys.argv) == 1:
        print("Usage examples:")
        print("  python add_odd_detections.py combined_file.json")
        print(
            "  python add_odd_detections.py combined_file.json odd_detections_folder/"
        )
        print(
            "  python add_odd_detections.py combined.json odd_folder/ -o enhanced_output.json"
        )
        print()
        print("Input structure:")
        print("  Combined file: List of entries with 'time_stamp' and 'object_list'")
        print(
            "  Odd folder: Contains JSON files with timestamp keys and 'pred'/'ood_score' values (default: odd_scores/)"
        )
        print()
        print("Output structure adds 'odd_detections' list to each timestamp entry.")
        print('Format: [{"cam": filename, "ood_score": value, "pred": true}]')
        print("Only includes entries where pred == true.")
        sys.exit(1)

    main()
