import os
import yaml

_yaml_path = os.path.join(
    os.path.dirname(__file__), "../config/robots.yaml"
)
with open(os.path.abspath(_yaml_path), "r") as f:
    _config = yaml.safe_load(f)

ALL_ROBOT_NAMES = _config["all_robot_names"]
ROBOT_NEIGHBORS = _config["robot_neighbors"]
