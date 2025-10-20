import numpy as np
import numpy as numpy
import pandas as pd
import sys
import argparse
import load_json_data
import json
sys.path.append('.')
import humanoid_config
from MultilabelPredictor import MultilabelPredictor

theta = np.pi / 2  # 90 degrees
save_path = 'models/'

# def preprocess_normalization(df):
#     print("RAW DATA:\n", df.shape, type(df) )
#     # POSITION NORMALIZATION: Make all points relative to one single point (MARKER_L_HIP)
#     # RESULT: As the result the x,y,z for MARKER_L_HIP becomes 0 
#     print("POSITION NORMALIZATION")
#     df_1 = df.copy()
#     for i in range(0, humanoid_config.NUM_MARKERS * 3 , 3):
#         df_1.iloc[:, i:i+3] = df.iloc[:, i:i+3] - df.iloc[:, (3*humanoid_config.MARKER_L_HIP):(3*humanoid_config.MARKER_L_HIP+3)].values
    
#     df_1.to_csv('NORM_1.csv', index=True)
#     return df_1


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
        df = pd.read_csv(csv_filename, index_col=0)

        # 1. SAMPLE OF DATASET (for testing)
        # subsample_size = 5000
        # df = df.sample(n=subsample_size, random_state=0)
        # time_limit = 2

        # 2. FULL DATASET (for actual training)
        time_limit = 500

        # train_data = preprocess_normalization(df)

        #X = df.drop(columns=humanoid_config.movable_joint_names)  # Source (features)
        #y = df[humanoid_config.movable_joint_names]               # Target (label)

        problem_types = ['regression'] * (humanoid_config.NUM_JOINTS) 


        hyperparameters = {
            'NN_TORCH': {}, 
            'GBM': {},
            'XGB':{}
        }

        print("time_limit", time_limit) # how many seconds to train the TabularPredictor for each label
        multi_predictor = MultilabelPredictor(labels=humanoid_config.movable_joint_names, problem_types=problem_types, path=save_path)
        # multi_predictor.fit(train_data, presets='best_quality',  ag_args_fit={'num_gpus': 1}, time_limit=time_limit, hyperparameters = hyperparameters)
        multi_predictor.fit(df, presets='best_quality',  ag_args_fit={'num_gpus': 1}, time_limit=time_limit, hyperparameters = hyperparameters)
    elif args.mode == 'eval':
        csv_filename = args.filename
        test_data = pd.read_csv(csv_filename, index_col=0)
        test_data_nolab = test_data.drop(columns=humanoid_config.movable_joint_names)
        multi_predictor = MultilabelPredictor.load(save_path)
        predictions = multi_predictor.predict(test_data_nolab)
        print("Predictions:  \n", predictions)
        evaluations = multi_predictor.evaluate(test_data)
        print(evaluations)
        print("Evaluated using metrics:", multi_predictor.eval_metrics)
        mse_values = [abs(evaluations[joint]['mean_squared_error']) for joint in humanoid_config.movable_joint_names]
        overall_mse = np.mean(mse_values)
        overall_rmse = np.sqrt(overall_mse)
    
        eval_results = {
            'evaluations': evaluations,
            'num_test_samples': len(test_data),
            'overall_mse': overall_mse,
            'overall_rmse': overall_rmse
        }
    
        with open('eval_results.json', 'w') as f:
            json.dump(eval_results, f, indent=2)
        print("Evaluation results saved to eval_results.json")
    
    elif args.mode == 'predict':
        p_val, p_key = load_json_data.load_pose_from_json(args.posefile)
        # df = preprocess_normalization(pd.DataFrame([p_val], columns=humanoid_config.pose_names))
        df = pd.DataFrame([p_val], columns=humanoid_config.pose_names)
        multi_predictor = MultilabelPredictor.load(save_path)
        predicted_motion = multi_predictor.predict(df)
        print("Predicted motion:\n", predicted_motion, type(predicted_motion) )
        data_dict = predicted_motion.iloc[0].to_dict()
        print(data_dict)
        with open(args.motionfile, 'w') as f:
            json.dump(data_dict, f, indent=2)