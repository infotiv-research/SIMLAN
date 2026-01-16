# Humanoid Camera System

This is a package that views the output of the cameras for the humanoid project.
The pkg is a legacy component from when the humanoid was its separate project, and it is probably wise to merge its content with another pkg.

## Setup and usage of the camera viewer.

The commands below is examples of how you can use the `camera_viewer.py`. It is used to save images of humanoids when generating humanoid datasets.

The arguments needed to run:

- `output_dir`: Folder where images and output is stored.
- `camera_ids`: What cameras to take into account. It uses "dataset_cameras" found in `config.sh`. Modify this variable to set what cameras `camera_viewer.py` should sample from.

```bash
ros2 run humanoid_camera_system camera_viewer.py  --ros-args -p output_dir:=$1 -p camera_ids:="${dataset_cameras}"
```
