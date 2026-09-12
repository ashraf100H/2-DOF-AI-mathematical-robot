"""Load the frozen ANN and reuse the robot/IK definitions from their notebooks."""
import ast
from hashlib import sha256
import json
from pathlib import Path

import joblib
import numpy as np
import pandas as pd
import tensorflow as tf


def _notebook_definition(path, name, definition_type):
    """Read one definition without executing training, plots, or other cells."""
    notebook = json.loads(path.read_text(encoding="utf-8"))
    prefix = "class " if definition_type is ast.ClassDef else "def "
    definitions = []
    for cell in notebook["cells"]:
        source = "".join(cell.get("source", []))
        if cell["cell_type"] == "code" and f"{prefix}{name}" in source:
            definitions.extend(node for node in ast.parse(source).body
                               if isinstance(node, definition_type) and node.name == name)
    if len(definitions) != 1:
        raise ValueError(f"Expected exactly one {name} definition in {path.name}.")
    return compile(ast.Module(body=definitions, type_ignores=[]), f"{path.name}:{name}", "exec")


def load_robot_pipeline(repository_root="."):
    """Return the existing robot and predict_angles(x, y) without retraining.

    Forward kinematics comes directly from the original mathematical notebook;
    prediction/scaling comes directly from the Step 5 helper definition.
    """
    root = Path(repository_root)
    metadata = json.loads((root / "models/training_metadata.json").read_text(encoding="utf-8"))
    data_path = root / "data/robot_ik_training.csv"
    if sha256(data_path.read_bytes()).hexdigest() != metadata["dataset_sha256"]:
        raise ValueError("The IK dataset does not match the frozen model metadata.")
    if metadata["features"] != ["x", "y"] or metadata["targets"] != ["theta1", "theta2"]:
        raise ValueError("Unexpected feature or target order.")

    input_scaler = joblib.load(root / "models/input_scaler.joblib")
    output_scaler = joblib.load(root / "models/output_scaler.joblib")
    if list(input_scaler.feature_names_in_) != metadata["features"]:
        raise ValueError("Input scaler feature order does not match metadata.")
    if list(output_scaler.feature_names_in_) != metadata["targets"]:
        raise ValueError("Output scaler target order does not match metadata.")

    namespace = {"np": np, "pd": pd, "features": metadata["features"],
                 "input_scaler": input_scaler, "output_scaler": output_scaler,
                 "model": tf.keras.models.load_model(root / "models/ik_ann.keras", compile=False)}
    exec(_notebook_definition(root / "forward_kinematics.ipynb", "RobotArm2DOF", ast.ClassDef), namespace)
    robot = namespace["RobotArm2DOF"](10, 10)
    exec(_notebook_definition(root / "ann_ik_validation.ipynb", "predict_angles", ast.FunctionDef), namespace)
    return robot, namespace["predict_angles"]
