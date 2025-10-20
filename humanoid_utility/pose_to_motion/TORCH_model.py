import numpy as np
import sys
import argparse
import load_json_data
import json
sys.path.append('.')
import humanoid_config
import pandas as pd
import torch
from torch.utils.data import Dataset, DataLoader
import torch.nn as nn

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model_path = 'models/PYTORCH.pth'

num_feat = -1 *(humanoid_config.NUM_JOINTS)
input_size = humanoid_config.NUM_MARKERS * 3 
output_size = humanoid_config.NUM_JOINTS
class MultiLabelRegressionDataset(Dataset):
    def __init__(self, csv_file):
        data = pd.read_csv(csv_file)
        
        self.features = torch.tensor(data.iloc[:, 1:num_feat].values, dtype=torch.float32)  # first column is index, then features
        self.labels = torch.tensor(data.iloc[:, num_feat:].values, dtype=torch.float32)  # Last num_feat columns as labels

    def __len__(self):
        return len(self.features)

    def __getitem__(self, idx):
        return self.features[idx], self.labels[idx]

# 2. Define a simple neural network
class RegressionModel(nn.Module):
    def __init__(self, input_dim, output_dim=1):
        super(RegressionModel, self).__init__()
        self.model = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Dropout(p=0.1),
            
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Dropout(p=0.1),
            
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Dropout(p=0.1),

            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Dropout(p=0.1),
            
            nn.Linear(128, 128),
            nn.ReLU(),
            
            nn.Linear(128, output_dim)
        )

    def forward(self, x):
        return self.model(x)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Model control")
    parser.add_argument(
        '--mode',
        type=str,
        choices=['train', 'predict', 'eval'],
        required=True,
        help="Mode in which to run the script: train, test, or evaluate"
    )
    parser.add_argument(
        '--filename',
        type=str,
        help="CSV file"
    )
    parser.add_argument(
        '--posefile',
        type=str,
        help="JSON file"
    )
    parser.add_argument(
        '--motionfile',
        type=str,
        help="JSON file"
    )
    args = parser.parse_args()

    if args.mode == 'train':
        csv_filename = args.filename
        dataset = MultiLabelRegressionDataset(csv_filename)
        dataloader = DataLoader(dataset, batch_size=32, shuffle=True)
        model = RegressionModel(input_size, output_size)
        criterion = nn.MSELoss()
        optimizer = torch.optim.Adam(model.parameters(), lr=0.0001)

        for epoch in range(20000):
            for inputs, targets in dataloader:
                outputs = model(inputs)
                loss = criterion(outputs, targets)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
            print(f"Epoch {epoch+1}, Loss: {loss.item():.4f}", flush=True)
        torch.save(model.state_dict(), model_path)
        print(f"Model saved to {model_path}")
    elif args.mode == 'predict':
        model = RegressionModel(input_size, output_size)
        model.load_state_dict(torch.load(model_path))
        model.eval()
        p_val, p_key = load_json_data.load_pose_from_json(args.posefile)
        pose_tensor = torch.tensor(p_val, dtype=torch.float32)
        with torch.no_grad():
            predicted_motion = model(pose_tensor)
            predicted_motion_dict = dict(zip(humanoid_config.movable_joint_names, predicted_motion.tolist()))
            with open(args.motionfile, 'w') as f:
                json.dump(predicted_motion_dict, f, indent=2)
    elif args.mode == 'eval':
        print("No implementer yet")