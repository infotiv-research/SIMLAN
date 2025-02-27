echo "<!-- Please update simulation/static_agent_launcher/description/camera_config.xacro with these cameras  -->"
echo "<!-- =================================== -->"
echo "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"
# python3  calibration.py intrinsic/160.yaml extrinsic/160.yaml 160
# python3  calibration.py intrinsic/161.yaml extrinsic/161.yaml 161
# python3  calibration.py intrinsic/162.yaml extrinsic/162.yaml 162
# python3  calibration.py intrinsic/163.yaml extrinsic/163.yaml 163
python3  calibration.py intrinsic/164.yaml extrinsic/164.yaml 164
# python3  calibration.py intrinsic/165.yaml extrinsic/165.yaml 165
# python3  calibration.py intrinsic/166.yaml extrinsic/166.yaml 166
# python3  calibration.py intrinsic/167.yaml extrinsic/167.yaml 167
# python3  calibration.py intrinsic/168.yaml extrinsic/168.yaml 168
# python3  calibration.py intrinsic/169.yaml extrinsic/169.yaml 169
# python3  calibration.py intrinsic/170.yaml extrinsic/170.yaml 170
# python3  calibration.py intrinsic/170.yaml extrinsic/171.yaml 171
echo "</robot>"