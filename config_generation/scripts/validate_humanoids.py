import json

REQUIRED_KEYS = ["namespace", "initial_pose_x", "initial_pose_y"]

def validate_humanoids(robots_str):
    try:
        robots = json.loads(robots_str)
    except json.JSONDecodeError as e:
        raise ValueError(f"\033[91m[ERROR] Failed to parse ROBOTS. Make sure ROBOTS is a valid list of dictionaries:\n{robots_str}\033[0m")

    try:
        if not isinstance(robots, list):
            raise ValueError(f"\033[91m[ERROR] Robots configuration is not a list, check config.sh\033[0m")
        
    except Exception as e:
        raise ValueError(f"\033[91m[ERROR]Failed to parse robots, robots should be a list of dictionaries: {e}, check config.sh\033[0m")

    for i, robot in enumerate(robots):
        if not isinstance(robot, dict):
            raise ValueError(f"\033[91m[ERROR] Invalid robot entry at index {i}: {robot}, check config.sh\033[0m")
        
        missing_keys = [k for k in REQUIRED_KEYS if k not in robot]
        if missing_keys:
            raise ValueError(f"\033[91m[ERROR] Robot '{robot.get('namespace', '?')}' is missing required keys: {missing_keys}, check config.sh\033[0m")
    return robots