#!/bin/bash
echo "<!-- Please update simulation/static_agent_launcher/description/camera_config.xacro with these cameras  -->"
echo "<!-- =================================== -->"
echo "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"


camera_ids=("$@")

# Run calibration for each camera
for camera_id in "${camera_ids[@]}"; do
    python3 calibration.py intrinsic/${camera_id}.yaml extrinsic/${camera_id}.yaml ${camera_id}
done

echo "</robot>"