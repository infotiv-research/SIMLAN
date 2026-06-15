#!/usr/bin/env python3
"""
Script to combine two JSON files and save the result with "_combined" suffix.
Merges prediction data into objects by matching track_id at the same timestamp.
The result has objects with combined data instead of separate predictions list.
"""

import json
import argparse
import os
from typing import Dict, List, Any


def load_json_file(filepath: str) -> List[dict]:
    """Load and return JSON data from file."""
    try:
        with open(filepath, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: File {filepath} not found.")
        return []
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON in file {filepath}.")
        return []


def combine_json_files(file1_data: List[dict], file2_data: List[dict], merge_mode: str = "timestamp") -> List[dict]:
    """
    Combine two JSON files.
    
    Args:
        file1_data: Data from first file (typically predictions)
        file2_data: Data from second file (typically object_list data)
        merge_mode: "timestamp" to merge by timestamp, "concatenate" to just combine all entries
    
    Returns:
        Combined data with predictions merged into objects by track_id
    """
    if merge_mode == "concatenate":
        # Simple concatenation
        return file1_data + file2_data
    
    elif merge_mode == "timestamp":
        # Merge by timestamp and track_id
        combined_dict = {}
        
        # First, process the file with object_list (usually the second file)
        for entry in file2_data:
            timestamp = entry.get("time_stamp")
            if timestamp is not None:
                combined_dict[timestamp] = {
                    "time_stamp": timestamp,
                    "object_list": entry.get("object_list", []).copy()
                }
        
        # Then process predictions file and merge into matching objects
        for entry in file1_data:
            timestamp = entry.get("time_stamp")
            predictions = entry.get("predictions", [])
            
            if timestamp is not None and timestamp in combined_dict:
                # Create a lookup for predictions by track_id
                predictions_by_id = {}
                for prediction in predictions:
                    track_id = prediction.get("track_id")
                    if track_id is not None:
                        predictions_by_id[track_id] = prediction.copy()
                        # Remove track_id from prediction data to avoid duplication
                        predictions_by_id[track_id].pop("track_id", None)
                
                # Merge prediction data into matching objects
                object_list = combined_dict[timestamp]["object_list"]
                for obj in object_list:
                    if isinstance(obj, dict):
                        track_id = obj.get("track_id")
                        if track_id is not None and track_id in predictions_by_id:
                            # Merge prediction data into the object
                            obj.update(predictions_by_id[track_id])
            
            elif timestamp is not None and timestamp not in combined_dict:
                # If timestamp doesn't exist in object_list file, create entry with just predictions
                combined_dict[timestamp] = {
                    "time_stamp": timestamp,
                    "object_list": []
                }
                # Convert predictions to objects
                for prediction in predictions:
                    obj = prediction.copy()
                    combined_dict[timestamp]["object_list"].append(obj)
        
        # Convert back to list and sort by timestamp
        combined_list = list(combined_dict.values())
        combined_list.sort(key=lambda x: x.get("time_stamp", 0))
        return combined_list
    
    else:
        raise ValueError(f"Unknown merge mode: {merge_mode}")


def combine_files(file1_path: str, file2_path: str, output_file: str = None, merge_mode: str = "timestamp"):
    """
    Combine two JSON files and save the result.
    In timestamp mode, merges prediction data into objects by matching track_id.
    
    Args:
        file1_path: Path to first JSON file (typically predictions file)
        file2_path: Path to second JSON file (typically object_list file)
        output_file: Output file path (default: first file with _combined suffix)
        merge_mode: How to combine the files ("timestamp" or "concatenate")
    """
    print(f"Loading first file: {file1_path}")
    file1_data = load_json_file(file1_path)
    
    print(f"Loading second file: {file2_path}")
    file2_data = load_json_file(file2_path)
    
    if not file1_data and not file2_data:
        print("Error: Both files are empty or could not be loaded.")
        return
    
    print(f"File 1 entries: {len(file1_data)}")
    print(f"File 2 entries: {len(file2_data)}")
    
    # Combine the data
    combined_data = combine_json_files(file1_data, file2_data, merge_mode)
    
    # Determine output file name
    if output_file is None:
        base_name, ext = os.path.splitext(file1_path)
        output_file = f"{base_name}_combined{ext}"
    
    # Save combined data
    try:
        with open(output_file, 'w') as f:
            json.dump(combined_data, f, indent=2)
        print(f"Combined data saved to: {output_file}")
        print(f"Combined entries: {len(combined_data)}")
        
        # Show some statistics
        if merge_mode == "timestamp":
            timestamps_file1 = set(entry.get("time_stamp") for entry in file1_data if entry.get("time_stamp"))
            timestamps_file2 = set(entry.get("time_stamp") for entry in file2_data if entry.get("time_stamp"))
            common_timestamps = timestamps_file1 & timestamps_file2
            print(f"Common timestamps: {len(common_timestamps)}")
            print(f"Unique timestamps from file 1: {len(timestamps_file1 - timestamps_file2)}")
            print(f"Unique timestamps from file 2: {len(timestamps_file2 - timestamps_file1)}")
            
            # Count merged objects
            total_merged_objects = 0
            for entry in combined_data:
                object_list = entry.get("object_list", [])
                total_merged_objects += len(object_list)
            print(f"Total merged objects: {total_merged_objects}")
            
    except Exception as e:
        print(f"Error saving combined file: {e}")


def main():
    parser = argparse.ArgumentParser(description="Combine two JSON files")
    parser.add_argument("file1", help="Path to first JSON file")
    parser.add_argument("file2", help="Path to second JSON file")
    parser.add_argument("-o", "--output", help="Output file path (default: file1_combined.json)")
    parser.add_argument("-m", "--mode", choices=["timestamp", "concatenate"], default="timestamp",
                        help="Merge mode: 'timestamp' merges by timestamp (default), 'concatenate' appends all entries")
    
    args = parser.parse_args()
    
    combine_files(args.file1, args.file2, args.output, args.mode)


if __name__ == "__main__":
    # Example usage when run directly
    import sys
    
    if len(sys.argv) == 1:
        print("Usage examples:")
        print("  python combine_json.py predictions.json ids.json")
        print("  python combine_json.py file1.json file2.json -o custom_output.json")
        print("  python combine_json.py file1.json file2.json -m concatenate")
        print()
        print("Merge modes:")
        print("  timestamp (default): Merge predictions into objects by track_id at same timestamp")
        print("  concatenate: Simply append all entries from both files")
        sys.exit(1)
    
    main()