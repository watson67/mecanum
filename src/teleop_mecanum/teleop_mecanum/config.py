import os
import yaml
from ament_index_python.packages import get_package_share_directory

package_share = get_package_share_directory('teleop_mecanum')
_yaml_path = os.path.join(package_share, 'config', 'robots.yaml')
with open(_yaml_path, "r") as f:
    _config = yaml.safe_load(f)

ROBOT_NAMES = _config["robot_names"]

