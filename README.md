# 2-DOF AI Mathematical Robot

## Project overview

This project develops a two-link planar robotic arm from mathematical modeling and simulation toward an intelligent physical robot. The goal is a complete manipulation pipeline that can determine a valid robot configuration, reach a target, plan a movement, grasp an object, and place it at a target location.

The initial 2D robot provides a controlled learning environment for principles that can later extend to higher-DOF, 3D robotic arms. Development is incremental:

**Mathematical modeling → Simulation → Configuration-space exploration → Workspace analysis → Inverse-kinematics policy → ML dataset → ANN → Motion planning → Pick and place → Hardware**

## Robot specification

| Parameter | Value |
|---|---|
| Link lengths | L1 = 10 cm, L2 = 10 cm |
| Dataset theta1 limits | −90° to 180° |
| Dataset theta2 limits | −120° to 120° |
| Sampling resolution | 1° per joint |
| Base | (0, 0); +X right, +Y up |
| Angle convention | theta1 counter-clockwise from +X; theta2 relative to Link 1 |

The notebook's validated `RobotArm2DOF.forward_kinematics()` is the source of truth for dataset generation and reconstruction checks. The earlier interactive sliders currently allow ±180° and the class does not enforce joint limits; the limits above describe the datasets.

## Progress and roadmap

| Step | Status |
|---|---|
| 1 — Mathematical model | ✅ Complete |
| 2 — Interactive simulation | ✅ Complete |
| 3.1 — Configuration-space generation | ✅ Complete |
| 3.2 — Workspace analysis | ✅ Complete |
| 3.3 — IK ambiguity resolution + ML dataset | ✅ Complete |
| 4 — TensorFlow ANN training | ✅ Complete |
| 5 — ANN inverse-kinematics validation | Next, after Step 4 review |
| 6 — Simulated box pickup | Planned |
| 7 — Table + trajectory planning | Planned |
| 8 — Gripper / complete 2D pick-and-place | Planned |
| Later — Physical hardware implementation | Planned |

## Project structure and datasets

```text
forward_kinematics.ipynb       # Robot model, simulation, data generation, Steps 3.2–3.3
ann_inverse_kinematics.ipynb   # Step 4: baseline ANN preparation, training, and diagnostics
requirements-ann.txt         # Tested ANN environment (Python 3.11)
data/
  robot_configurations.csv    # Complete raw configuration-space dataset
  robot_ik_training.csv       # Deterministic IK samples for future ANN training
models/
  ik_ann.keras               # Trained baseline with best validation weights
  input_scaler.joblib        # Train-fitted position scaler
  output_scaler.joblib       # Train-fitted angle scaler
  split_indices.npz          # Fixed train/validation/test row indices
  training_history.csv       # Per-epoch scaled MSE and MAE
  training_metadata.json     # Dataset hash, versions, settings, and results
```

- **Raw dataset:** 65,311 rows, columns `theta1, theta2, x, y`. Preserve this ground-truth dataset for analysis and future configuration policies.
- **IK training dataset:** 39,555 rows, columns `x, y, theta1, theta2`. Future ANN inputs are `[x, y]` (cm); outputs are `[theta1, theta2]` (degrees). No analysis helper columns are included.

The raw data contains **25,756 Cartesian groups with two valid configurations**. Direct regression on both labels could average incompatible joint configurations. Step 3.3 groups coordinates rounded to six decimals in cm, checks grouping stability and valid paired configurations, and retains original coordinate values in the final CSV.

Our **first IK policy** prefers positive theta2 (elbow-down for a target on +X), keeps negative theta2 when it is the only valid branch, and retains straight configurations. Positive-only selection would lose 6,764 reachable targets under the joint limits. The fallback preserves all sampled targets: 32,520 positive, 271 straight, and 6,764 negative configurations.

All selected targets are unique at the grouping precision and reconstruct within 1e-9 cm using the existing forward-kinematics method. X and Y each span −20 to +20 cm; radial reach spans 10 to 20 cm. The workspace is not a filled disk or complete annulus.

This policy can introduce discontinuities where selection switches branches. Unique labels do not guarantee easy ANN regression. Step 5 must assess the trained model near these boundaries. A future state-aware policy may use the current robot configuration to minimize movement and support safe trajectories; it is not implemented yet.

## Step 4 baseline results

The ANN uses only `robot_ik_training.csv`: inputs `[x, y]` (cm), outputs `[theta1, theta2]` (degrees). Seed 42 produces **31,644 training / 3,955 validation / 3,956 reserved test** rows. Separate StandardScalers for inputs and targets are fitted on training rows only. The current IK policy is unchanged.

The network is **2 → Dense(64, ReLU) → Dense(64, ReLU) → Dense(2, linear)** with **4,482 trainable parameters**. It uses Adam (0.001), scaled MSE loss, scaled MAE monitoring, and batch size 128. Early stopping with patience 20 ended training at **207 epochs** and restored **epoch 187**, with best validation MSE **0.00440394**. Restored-model validation MAEs are **1.7447° for theta1** and **2.1763° for theta2** (overall **1.9605°**).

Training and validation errors decrease substantially, then validation improvements level off with some fluctuations. There is no sustained validation-error rise indicating clear overfitting. Remaining angle errors and policy discontinuities still require Step 5 investigation; random-split validation describes interpolation within this sampled workspace. The test set has not been evaluated, and Cartesian forward-kinematics validation has not started.

## Running the notebook

Open the notebook with its working directory set to the repository root. It uses NumPy, Pandas, Matplotlib, IPython/Jupyter, and `ipympl` for interactive sliders.

To rerun analysis without regenerating raw data, execute the NumPy, `RobotArm2DOF`, and `robot` definition cells, then Step 3.2 or Step 3.3. Step 3.3 reads the raw CSV and writes only `data/robot_ik_training.csv`. Earlier generation cells intentionally regenerate the raw CSV when run. Saved notebook outputs show the analysis without requiring the interactive backend.

For ANN work, use a dedicated Python 3.11 environment, install `requirements-ann.txt`, select that environment's Jupyter kernel, and open `ann_inverse_kinematics.ipynb`. Section 4.8 shows how to load the model and scalers for predictions without retraining. Reuse `split_indices.npz` in Step 5 only after verifying the dataset SHA256 recorded in `training_metadata.json`; indices are zero-based data-row positions after the CSV header. Numerical history and software versions are saved with the model.

**Review Step 4 before continuing to Step 5.**
