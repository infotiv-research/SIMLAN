## Autogluon Model configuration

This section describes how AutoGluon is configured for a multi-label regression problem in our current setup.

To use AutoGluon for a multi-label regression problem, first we need to create a MultilabelPredictor by setting

- **labels**: the labels that we want to predict.
- **problem_types**: the problem type for each TabularPredictor.
- **path**: path to the directory where models and intermediate outputs should be saved.
- **consider_labels_correlation**: Whether the predictions of multiple labels should account for label correlations or predict each label independently of the others.

For **consider_labels_correlation**, we set it to FALSE in order to disable using one label as a feature for another. The reasons for this are:

- Each joint has its own degree of freedom and control, and it is independent from the rest of the joints.
- Training stability without error propagation.
- When set to False, the training will be faster.
- More general and simple model.

For training the model, we should take into consideration the following hyperparameters related to:

- Stacking: which is an ensemble technique where multiple models are trained and then a "meta-model" learns how to combine their predictions. If we use dynamic Stacking, it will automatically determine the optimal number of stacking layers and which models to include.
- Bagging: multiple training versions of the same model on different subsets of data and combining their predictions.
- preset: which condenses the complex hyperparameter setups. For example, we can use a small model size by using medium_quality, which will lead to faster training but with less prediction quality. Or we can use a large model by setting preset to best_quality, which will lead to better performance but much longer training time.

The current AutoGluon uses the following hyperparameters:

- **dynamic_stacking=False**: Disables the automatic stacking optimization.
- **num_stack_levels=0**: no stacking level.
- **auto_stack=False**: Disables automatic ensemble stacking.
- **num_bag_folds=0** and **num_bag_sets=1**, which means no bagging.

So here is the current training flow for each joint:

- Train NN_TORCH, GBM, and XGB models.
- No bagging: Each model trains on the full dataset once, no multiple training versions of the same model on different subsets of data and combining their predictions.
- No stacking: No meta-models combining predictions.
- Best model selection: Choose the best performing model per joint.

**Eliminating dynamic stacking and multi-level stacking will lead to 50% to 70% time savings.**

Overall, this configuration prioritizes training speed and stability over ensemble complexity, making it suitable for fast iteration and independent joint predictions.

## Hyperparameter Tuning

In the current AutoGluon model, we are training NN_TORCH, GBM, and XGB models by using their default built-in hyperparameter settings.
The next step for performance improvement is to add the **hyperparameter_tune_kwargs argument** to enable AutoGluon’s internal hyperparameter optimization.
This will allow AutoGluon to automatically search for the best hyperparameters for each model type, balancing training time and prediction quality.
