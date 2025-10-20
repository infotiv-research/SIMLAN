import re

def update_pose_y_value(file_path):
    with open(file_path, 'r') as f:
        content = f.read()

    def update_pose(match):
        pose_str = match.group(1)
        parts = pose_str.strip().split()
        
        if len(parts) != 6:
            print(f"Skipping malformed pose: {pose_str}")
            return f"<pose>{pose_str}</pose>"

        try:
            # Convert all to float
            values = [float(p) for p in parts]
            # Add +2 to the Y value (index 1)
            values[1] += 0.1
            # Format back to string with up to 6 decimals
            new_pose = ' '.join(f"{v:.6f}".rstrip('0').rstrip('.') if '.' in f"{v:.6f}" else f"{v:.6f}" for v in values)
            return f"<pose>{new_pose}</pose>"
        except ValueError:
            print(f"Skipping invalid float in pose: {pose_str}")
            return f"<pose>{pose_str}</pose>"

    # Regex to match <pose>...</pose>
    updated_content = re.sub(r"<pose>(.*?)</pose>", update_pose, content, flags=re.DOTALL)

    # write to file
    with open("/home/ros/src/simulation/simlan_gazebo_environment/worlds/ign_simlan_factory_updated.world", 'w') as f:
        f.write(updated_content)

    print("Pose Y-values updated and saved to:", "updated_" + file_path)


update_pose_y_value("/home/ros/src/simulation/simlan_gazebo_environment/worlds/ign_simlan_factory_updated.world")