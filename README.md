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
| 4 — TensorFlow ANN training | Next, after Step 3.3 review |
| 5 — ANN inverse-kinematics validation | Planned |
| 6 — Simulated box pickup | Planned |
| 7 — Table + trajectory planning | Planned |
| 8 — Gripper / complete 2D pick-and-place | Planned |
| Later — Physical hardware implementation | Planned |

## Project structure and datasets

```text
forward_kinematics.ipynb       # Robot model, simulation, data generation, Steps 3.2–3.3
data/
  robot_configurations.csv    # Complete raw configuration-space dataset
  robot_ik_training.csv       # Deterministic IK samples for future ANN training
```

- **Raw dataset:** 65,311 rows, columns `theta1, theta2, x, y`. Preserve this ground-truth dataset for analysis and future configuration policies.
- **IK training dataset:** 39,555 rows, columns `x, y, theta1, theta2`. Future ANN inputs are `[x, y]` (cm); outputs are `[theta1, theta2]` (degrees). No analysis helper columns are included.

The raw data contains **25,756 Cartesian groups with two valid configurations**. Direct regression on both labels could average incompatible joint configurations. Step 3.3 groups coordinates rounded to six decimals in cm, checks grouping stability and valid paired configurations, and retains original coordinate values in the final CSV.

Our **first IK policy** prefers positive theta2 (elbow-down for a target on +X), keeps negative theta2 when it is the only valid branch, and retains straight configurations. Positive-only selection would lose 6,764 reachable targets under the joint limits. The fallback preserves all sampled targets: 32,520 positive, 271 straight, and 6,764 negative configurations.

All selected targets are unique at the grouping precision and reconstruct within 1e-9 cm using the existing forward-kinematics method. X and Y each span −20 to +20 cm; radial reach spans 10 to 20 cm. The workspace is not a filled disk or complete annulus.

This policy can introduce discontinuities where selection switches branches. Unique labels do not guarantee easy ANN regression. Step 4 still needs splitting/scaling and later validation near these boundaries. A future state-aware policy may use the current robot configuration to minimize movement and support safe trajectories; it is not implemented yet.

## Running the notebook

Open the notebook with its working directory set to the repository root. It uses NumPy, Pandas, Matplotlib, IPython/Jupyter, and `ipympl` for interactive sliders.

To rerun analysis without regenerating raw data, execute the NumPy, `RobotArm2DOF`, and `robot` definition cells, then Step 3.2 or Step 3.3. Step 3.3 reads the raw CSV and writes only `data/robot_ik_training.csv`. Earlier generation cells intentionally regenerate the raw CSV when run. Saved notebook outputs show the analysis without requiring the interactive backend.

**TensorFlow training has not started. Review Step 3.3 before continuing.**
