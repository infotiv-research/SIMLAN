#!/usr/bin/env python3
"""
Convert CSV format from long to wide format for multi-camera pose data.
Usage: python csvtowide.py <input.csv> <output_wide.csv>
"""

import pandas as pd
import sys
import os

def convert_to_wide(input_csv, output_csv):
    """Convert regular CSV to wide format"""
    # Read the CSV file
    df = pd.read_csv(input_csv)
    print('original_shape', df.shape)
    
    # Identify columns
    pose_cols = [c for c in df.columns if '_' in c and c.split('_')[0].isdigit()]
    motion_cols = [c for c in df.columns if c not in ['motion_id', 'camera_id'] + pose_cols]
    print('pose_cols', len(pose_cols), pose_cols[:5] if len(pose_cols) > 5 else pose_cols)
    print('motion_cols', len(motion_cols), motion_cols[:5] if len(motion_cols) > 5 else motion_cols)
    
    # Pivot to wide format
    wide = df.pivot(index='motion_id', columns='camera_id', values=pose_cols)
    wide.columns = [f'cam{cam}_{feat}' for feat, cam in wide.columns.to_flat_index()]
    wide = wide.reset_index()
    
    # Merge with motion data
    motion_df = df.drop_duplicates('motion_id')[['motion_id'] + motion_cols]
    result = pd.merge(wide, motion_df, on='motion_id', how='left')
    
    # Sort by motion_id to maintain order
    result = result.sort_values('motion_id')
    
    # Remove motion_id column from final output
    result = result.drop('motion_id', axis=1)
    
    # Save to CSV without motion_id
    result.to_csv(output_csv, index=False)
    print('final_shape', result.shape)
    print(f'Converted {input_csv} -> {output_csv}')
    print('Sample columns:', result.columns[:10].tolist())

def main():
    if len(sys.argv) != 3:
        sys.exit(1)
    
    input_csv = sys.argv[1]
    output_csv = sys.argv[2]
    
    if not os.path.exists(input_csv):
        print(f"Error: Input file {input_csv} not found")
        sys.exit(1)
    
    convert_to_wide(input_csv, output_csv)

if __name__ == "__main__":
    main()