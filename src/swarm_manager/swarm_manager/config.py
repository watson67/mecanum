import os
import yaml
from ament_index_python.packages import get_package_share_directory

package_share = get_package_share_directory('mecanum_swarm')
_yaml_path = os.path.join(package_share, 'config', 'robots.yaml')
with open(_yaml_path, "r") as f:
    _config = yaml.safe_load(f)

ALL_ROBOT_NAMES = _config["all_robot_names"]
ROBOT_NEIGHBORS = _config["robot_neighbors"]
