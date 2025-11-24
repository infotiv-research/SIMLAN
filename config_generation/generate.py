import sys

from scripts.validate_robots import validate_robots
from scripts.validate_humanoids import validate_humanoids
from scripts.generate_bt_xml import generate_bt_xml
from scripts.generate_control_yaml import generate_control_yaml
from scripts.generate_gz_bridge_yaml import generate_gz_bridge
from scripts.generate_nav2_params_yaml import generate_nav2_params
from scripts.generate_humanoid_control_yaml import generate_humanoid_control_yaml
from scripts.generate_humanoid_nav2_params_yaml import generate_humanoid_nav2_params


def main():
    robots_index = sys.argv.index("--robots") + 1
    robots_str = sys.argv[robots_index]
    humanoids_index = sys.argv.index("--humanoids") + 1
    humanoids_str = sys.argv[humanoids_index]

    ##validation##
    robots = validate_robots(robots_str)
    humanoids = validate_humanoids(humanoids_str)
    ##robot_generation##
    generate_bt_xml(robots)
    generate_control_yaml(robots)
    generate_gz_bridge(robots, humanoids)
    generate_nav2_params(robots)
    ##humanoid_generation##
    generate_humanoid_control_yaml(humanoids)
    generate_humanoid_nav2_params(humanoids)


if __name__ == "__main__":
    main()
