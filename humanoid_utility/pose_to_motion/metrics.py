import torch


# Runs evaluation loop, given a test filename and a model. Return MSE and MAE.
def eval(predictions, targets):

    # Convert to torch tensors
    preds = torch.stack(predictions)
    trues = torch.stack(targets)

    # Compute metrics
    mse = torch.mean((preds - trues) ** 2).item()
    mae = torch.mean(torch.abs(preds - trues)).item()

    print(f"MSE: {mse}, MAE: {mae}")
    return mse, mae
