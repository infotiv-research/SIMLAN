import json
import sys
import argparse

sys.path.append(".")

from humanoid_utility.pose_to_motion.generate_report import generate_eval_report, generate_train_report
from humanoid_utility import humanoid_config
from humanoid_utility.pre_processing.load_json_data import load_json_data, load_pose_from_json

from humanoid_utility.pose_to_motion.metrics import run_evaluation
from humanoid_utility.pose_to_motion.pytorch.utils import HyperparameterDict, MultiLabelRegressionDataset,  get_hyperparameters, load_model, save_model
from torch.utils.data import random_split
sys.path.append(".")
import torch
from torch.utils.data import DataLoader
import torch.nn as nn
import optuna
from optuna.trial import TrialState
import torch.optim as optim
from sklearn.metrics import mean_squared_error
from pathlib import Path


def objective(trial: optuna.Trial, train_filename):
    # Hyperparams
    ###############################
    N_EPOCHS = trial.suggest_int("N_EPOCHS", 1, 100)
    BATCHSIZE = trial.suggest_int("BATCHSIZE", 12, 512)
    n_layers = trial.suggest_int("n_layers", 1, 8)
    size_per_layer = [
        trial.suggest_int("n_units_l{}".format(i), 1, 512) for i in range(n_layers)
    ]
    dropout_per_layer = [
        trial.suggest_float("dropout_l{}".format(i), 0.1, 0.9) for i in range(n_layers)
    ]
    optimizer_name = trial.suggest_categorical("optimizer", ["Adam", "RMSprop", "SGD"])
    lr = trial.suggest_float("lr", 1e-5, 1e-1, log=True)
    ################################
    hyperparameters = HyperparameterDict({
        # TRAIN LOOP SPECIFIC
        "N_EPOCHS": N_EPOCHS,
        "BATCHSIZE": BATCHSIZE,
        "optimizer_name": optimizer_name,
        "lr":lr,
        "n_layers": n_layers,
        # MODEL SPECIFIC
        "size_per_layer": size_per_layer,
        "dropout_per_layer": dropout_per_layer,
        "in_features": humanoid_config.NUM_MARKERS * 3,
        "out_features": humanoid_config.NUM_JOINTS
    })
    # Train feedback loop. Can choose if use trial or not.
    average_validation_mse, _ = train_loop(
        train_filename,
        hyperparameters,
        "optuna",
        trial=trial,
    )
    return average_validation_mse


def train_optuna(train_filename):

    study_name = "test_if_works-study"  # Unique identifier of the study.
    study_save_location = Path(__file__).parent / "output/optuna_studies" / study_name

    storage_name = "sqlite:///{}.db".format(study_save_location)
    study = optuna.create_study(
        study_name=study_name,
        storage=storage_name,
        direction="minimize",
        load_if_exists=True,
    )
    study.optimize(
        lambda trial: objective(trial, train_filename),
        n_trials=10000
    )
    pruned_trials = study.get_trials(deepcopy=False, states=[TrialState.PRUNED])
    complete_trials = study.get_trials(deepcopy=False, states=[TrialState.COMPLETE])

    print("Study statistics: ")
    print("  Number of finished trials: ", len(study.trials))
    print("  Number of pruned trials: ", len(pruned_trials))
    print("  Number of complete trials: ", len(complete_trials))

    print("Best trial:")
    trial = study.best_trial

    print("  Value: ", trial.value)

    print("  Params: ")
    for key, value in trial.params.items():
        print("    {}: {}".format(key, value))


def train_normal(train_filename, model_instance ):
    
    # Get hyperparams
    hyperparameters = get_hyperparameters()

    # Train feedback loop. Can choose if use trial or not.
    val_mse, model_instance = train_loop(
        train_filename,
        hyperparameters,
        model_instance,
        trial=None,
    )
    generate_train_report("pytorch",  model_instance, train_filename, val_mse, " ", str(hyperparameters) )
    
def train_loop(  
    train_filename,
    hyperparameters,
    model_instance,
    trial=None,
):
   
    # Generate the model.
    DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
    # Load model checks if name exists and loads that one else it creates a new one.
    model = load_model(model_instance, hyperparameters)
    model.to(DEVICE)
    # Criterion
    criterion = nn.MSELoss()

    # Generate the optimizers.
    optimizer = getattr(optim, hyperparameters["optimizer_name"])(model.parameters(), lr=hyperparameters["lr"])

    # Create train and validation dataset by splitting with set ratio.
    dataset = MultiLabelRegressionDataset(train_filename, hyperparameters["in_features"])
    train_val_split = 0.8
    train_size = int(len(dataset) * train_val_split)
    valid_size = len(dataset) - train_size
    train_dataset, validation_dataset = random_split(
        dataset, [train_size, valid_size],
        generator=torch.Generator().manual_seed(42)  # for reproducibility
    )
    
    train_loader = DataLoader(train_dataset, batch_size=hyperparameters["BATCHSIZE"], shuffle=False)
    valid_loader = DataLoader(validation_dataset, batch_size=128, shuffle=False)

    # Training of the model.
    for epoch in range(hyperparameters["N_EPOCHS"]):
        model.train()
        mse_total = 0.0

        for data, target in train_loader:

            data, target = data.view(data.size(0), -1).to(DEVICE), target.to(DEVICE)

            optimizer.zero_grad()
            output = model(data)
            loss = criterion(output, target)
            loss.backward()
            optimizer.step()

        # Validation of the model.
        model.eval()
        with torch.no_grad():
            for data, target in valid_loader:

                data, target = data.view(data.size(0), -1).to(DEVICE), target.to(DEVICE)
                predictions_cpu = model(data).detach().cpu().numpy()
                target_cpu = target.detach().cpu().numpy()
                mse = mean_squared_error(predictions_cpu, target_cpu)

                mse_total += mse
        mse_average = mse_total / len(valid_loader)

        print(f"Epoch {epoch+1} / {hyperparameters["N_EPOCHS"]}, MSE: {mse_average:.8f}", flush=True)

    # After we are done training we save the model.
    model_instance = save_model(model, model_instance)
        
    if trial is not None:
        trial.report(mse_average, epoch)
        # Handle pruning based on the intermediate value.
        if trial.should_prune():
            raise optuna.exceptions.TrialPruned()

    return mse_average, model_instance

def run_setup_evaluation(eval_filename, model_instance):

    # Get hyperparams
    hyperparameters = get_hyperparameters()
    # Load saved model
    model = load_model(model_instance, hyperparameters)
    # Run eval and get metrics
    average_mse, average_mae = run_evaluation(eval_filename, model, hyperparameters)
    # We finish by generating a report of the run
    generate_eval_report(
        "pytorch",
        model_instance,
        eval_filename, 
        average_mse, 
        average_mae, 
        hyperparameters
        ) 


def generate_output_dirs():
    
    # Base output paths
    output_folder = Path(__file__).parent / "output"
    reports_folder = output_folder / "reports"
    model_states_folder = output_folder / "saved_model_states"

    # Create directories (including parents)
    output_folder.mkdir(parents=True, exist_ok=True)
    reports_folder.mkdir(parents=True, exist_ok=True)
    model_states_folder.mkdir(parents=True, exist_ok=True)

    # Verify existence
    assert model_states_folder.is_dir(), "Model states dir not found or could not be generated"
    assert reports_folder.is_dir(), "Reports dir not found or could not be generated"
        

def single_prediction(model_instance, pose_filename, motion_filename):
  
    p_val, p_key = load_pose_from_json(pose_filename)
    
    hyperparameters = get_hyperparameters()
    # Load saved model
    model = load_model(model_instance, hyperparameters)    
    predicted_motion = model(torch.Tensor(p_val))
    print("Predicted motion:\n", predicted_motion, type(predicted_motion))
    data_dict = dict(zip(humanoid_config.movable_joint_names, predicted_motion.detach().tolist()))
    with open(motion_filename, "w") as f:
        json.dump(data_dict, f, indent=2)

def main(args):

    generate_output_dirs()

    if args.mode == "single_train":
        train_normal(args.dataset_filename, args.model_instance)
    elif args.mode == "multi_train":
        raise NotImplementedError    
    elif args.mode == "optuna":
        train_optuna(args.dataset_filename)
    elif args.mode == "eval":
        run_setup_evaluation(args.dataset_filename, args.model_instance)
    
    elif args.mode == "single_predict":
        single_prediction(args.model_instance, args.pose_file, args.motion_file)
    elif args.mode == "multi_predict":
        raise NotImplementedError    

if __name__ == "__main__":
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
        help="Mode in which to run the script: train, eval, or evaluate",
    )

    parser.add_argument("--dataset_filename", type=str, help="CSV file")
    parser.add_argument("--model_instance", type=str, help="Model name to either create or load", default="")
    

    # Args for multi prediction
    parser.add_argument("--sample_root", type=str, help="Root that contains cameras directories")
    parser.add_argument(
        "--sample_id", type=str, help="used to find the pose and motion json files"
    )
    # Args for single prediction
    parser.add_argument("--motion_file", type=str, help="JSON file. predicted motion to save")
    parser.add_argument("--pose_file", type=str, help="JSON file, pose to load")

    args = parser.parse_args()
    main(args)
