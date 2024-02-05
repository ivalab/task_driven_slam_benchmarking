# closedloop_nav_slam

## Evaluation

Once the experiments are done, the saved results can be evaluated through the evaluation tool.

### Install Dependencies

  - [evo package](https://github.com/MichaelGrupp/evo?tab=readme-ov-file)
  
        pip install evo --upgrade --no-binary evo

### Set Parameters

1. First the running config file [config.yaml](../settings/config.yaml) defines the dataset and paths to be evaluated.

2. The evaluation config file [config.yaml](config.yaml) defines the methods to be evaluated.
    1. You can decide whether to save the evaluation results to the same running result directory.
    2. You can choose to load the saved evaluation results for plotting instead of re-evaluating from scratch.

### Run Evaluation
        
    python run_evaluation.py