Evaluation tool for **Task-driven SLAM benchmarking for Robot Navigation**.

## Instruction

Once the experiments are done, the results can be evaluated through this evaluation tool.

## Installation

- Dependencies

    - [evo package](https://github.com/MichaelGrupp/evo?tab=readme-ov-file)
  
            pip install evo --upgrade --no-binary evo
    - [apriltag](google.com)

            pip install apritag

- Install the project

        pip install -e .

## Evaluation

### 1. Closedloop
#### a. Simulation (Gazebo)

- Set the Parameters

    - [params.yaml](cl_sim/config/params.yaml)

- Run the Evaluation
```bash
# Evaluation and visualization per each sequence (env,path).
python cl_sim/main.py

# Visualize all sequences.
pythoh cl_sim/vis_all_sequences.py

# Visualize in modes (w/ and w/o map).
pythoh cl_sim/vis_all_sequences.py

```

#### b. Real World (TSRB Office)

- Set the Parameters

    - [params.yaml](cl_real/config/params.yaml)

- Run the Evaluation

```bash
# Evaluation and visualization per each sequence (env,path).
python cl_real/main.py

# Visualize in modes (w/ and w/o map).
pythoh cl_real/vis_in_modes.py

```

### 2. Openloop (EuRoC)

- Run the Evaluation

        python ol_real/main.py