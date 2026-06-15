#!/usr/bin/env python3
"""
Script to remove named fields from objects in a JSON file.
Can handle nested objects within arrays in the JSON structure.
"""

import json
import argparse
import os
from typing import Dict, List, Any


def remove_field_from_objects(data: Any, field_to_remove: str, container_key: str = None) -> int:
    """
    Remove a named field from objects in the data structure.
    
    Args:
        data: The JSON data (list or dict)
        field_to_remove: Name of the field to remove
        container_key: Optional key that contains the object list (e.g., 'object_list', 'predictions')
    
    Returns:
        Number of fields removed
    """
    removed_count = 0
    
    if isinstance(data, list):
        # Handle list of entries (top level)
        for entry in data:
            if isinstance(entry, dict):
                if container_key and container_key in entry:
                    # Remove from specific container (e.g., object_list, predictions)
                    container = entry[container_key]
                    if isinstance(container, list):
                        for obj in container:
                            if isinstance(obj, dict) and field_to_remove in obj:
                                del obj[field_to_remove]
                                removed_count += 1
                else:
                    # Remove directly from entry if no container specified
                    if field_to_remove in entry:
                        del entry[field_to_remove]
                        removed_count += 1
                    
                    # Also check all nested lists and objects
                    removed_count += remove_field_from_nested(entry, field_to_remove)
    
    elif isinstance(data, dict):
        # Handle single dict object
        if container_key and container_key in data:
            container = data[container_key]
            if isinstance(container, list):
                for obj in container:
                    if isinstance(obj, dict) and field_to_remove in obj:
                        del obj[field_to_remove]
                        removed_count += 1
        else:
            # Remove from the dict itself
            if field_to_remove in data:
                del data[field_to_remove]
                removed_count += 1
            
            # Check nested structures
            removed_count += remove_field_from_nested(data, field_to_remove)
    
    return removed_count


def remove_field_from_nested(obj: Dict[str, Any], field_to_remove: str) -> int:
    """Recursively remove field from nested structures."""
    removed_count = 0
    
    for key, value in list(obj.items()):
        if isinstance(value, list):
            for item in value:
                if isinstance(item, dict) and field_to_remove in item:
                    del item[field_to_remove]
                    removed_count += 1
        elif isinstance(value, dict):
            if field_to_remove in value:
                del value[field_to_remove]
                removed_count += 1
            # Recursively check nested dicts
            removed_count += remove_field_from_nested(value, field_to_remove)
    
    return removed_count


def clean_json_file(input_file: str, field_to_remove: str, container_key: str = None, output_file: str = None):
    """
    Clean JSON file by removing specified field from objects.
    
    Args:
        input_file: Path to input JSON file
        field_to_remove: Name of field to remove
        container_key: Optional container key (e.g., 'object_list', 'predictions')
        output_file: Optional output file path (defaults to overwriting input)
    """
    # Load JSON file
    try:
        with open(input_file, 'r') as f:
            data = json.load(f)
        print(f"Loaded JSON file: {input_file}")
    except FileNotFoundError:
        print(f"Error: File '{input_file}' not found.")
        return
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in '{input_file}': {e}")
        return
    
    # Remove the specified field
    removed_count = remove_field_from_objects(data, field_to_remove, container_key)
    
    # Determine output file
    if output_file is None:
        output_file = input_file
    
    # Save the cleaned data
    try:
        with open(output_file, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Saved cleaned JSON to: {output_file}")
        print(f"Removed {removed_count} instances of field '{field_to_remove}'")
    except Exception as e:
        print(f"Error saving file: {e}")


def main():
    parser = argparse.ArgumentParser(description="Remove named fields from objects in a JSON file")
    parser.add_argument("input_file", help="Path to input JSON file")
    parser.add_argument("field_name", help="Name of the field to remove")
    parser.add_argument("-c", "--container", help="Container key that holds the objects (e.g., 'object_list', 'predictions')")
    parser.add_argument("-o", "--output", help="Output file path (default: overwrite input file)")
    parser.add_argument("--backup", action="store_true", help="Create backup of original file")
    
    args = parser.parse_args()
    
    # Create backup if requested
    if args.backup:
        # Insert '_backup' before file extension
        base_name, ext = os.path.splitext(args.input_file)
        backup_file = f"{base_name}_backup{ext}"
        try:
            import shutil
            shutil.copy2(args.input_file, backup_file)
            print(f"Created backup: {backup_file}")
        except Exception as e:
            print(f"Warning: Could not create backup: {e}")
    
    # Clean the JSON file
    clean_json_file(args.input_file, args.field_name, args.container, args.output)


if __name__ == "__main__":
    # Example usage when run directly
    import sys
    
    if len(sys.argv) == 1:
        print("Usage examples:")
        print("  python json_cleaner.py data.json field_name")
        print("  python json_cleaner.py data.json field_name -c object_list")
        print("  python json_cleaner.py data.json field_name -c predictions -o cleaned_data.json")
        print("  python json_cleaner.py data.json field_name --backup")
        print()
        print("For your specific use cases:")
        print("  # Remove field from objects in object_list:")
        print("  python json_cleaner.py ids.json field_name -c object_list")
        print("  # Remove field from predictions:")
        print("  python json_cleaner.py predictions.json track_id -c predictions")
        sys.exit(1)
    
    main()