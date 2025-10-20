import sys

from scripts.validate_robots import validate_robots
from scripts.generate_bt_xml import generate_bt_xml
from scripts.generate_control_yaml import generate_control_yaml
from scripts.generate_gz_bridge import generate_gz_bridge
from scripts.generate_nav2_params import generate_nav2_params

def main():
    robots_index = sys.argv.index("--robots") + 1
    robots_str = sys.argv[robots_index]
    
    robots=validate_robots(robots_str)

    generate_bt_xml(robots)
    generate_control_yaml(robots)
    generate_gz_bridge(robots)
    generate_nav2_params(robots)

if __name__=="__main__":
    main()