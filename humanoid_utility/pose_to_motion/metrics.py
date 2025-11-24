
from humanoid_utility.pose_to_motion.pytorch.utils import MultiLabelRegressionDataset
import torch
from torch.utils.data import  DataLoader
from sklearn.metrics import mean_squared_error
from tqdm import tqdm


# Runs evaluation loop, given a test filename and a model. Return MSE and MAE.
def run_evaluation(test_filename, model, hyperparameters):
    
    DEVICE = "cpu"

    ## Dataset
    test_dataset = MultiLabelRegressionDataset(test_filename, hyperparameters["in_features"])
    test_loader = DataLoader(test_dataset, batch_size=hyperparameters["BATCHSIZE"], shuffle=False)

    mse_total = 0.0
    mae_total = 0.0
    
    ## run eval loop
    with torch.no_grad():
        for data, target in tqdm(test_loader):
            data, target = data.view(data.size(0), -1).to(DEVICE), target.to(DEVICE)
            prediction = model(data)
            mse = mean_squared_error(target, prediction)
    
            mse_total += mse

            mae_total += torch.mean(torch.abs(prediction - target)).item()
        mae_average = mae_total / len(test_loader)
        mse_average = mse_total / len(test_loader)
        print(
            f"total MSE: {mse_total}, average MSE is {mse_average}, total MAE is: {mae_total}, average MAE is: {mae_average}"
        )
    return mse_average, mae_average