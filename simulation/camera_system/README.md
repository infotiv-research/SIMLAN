# Camera_system

This is a package that views the output of the cameras for the humanoid project. At the moment, it is hardcoded to listen to the camera IDs 500-503 in the camera_viewer.py script.

The pkg is a legacy component from when the humanoid was its separate project, and it is probably wise to merge its content with another pkg.

The launch file should probably be removed, but since I'm leaving this project soon, I don't want to create any issues if that's not the case.

However, it is important to understand that the camera_viewer.py is run in the dataset function in control.sh

```bash
camera () {
    echo "::: CAMERA  :::"
    source install/setup.bash ; ros2 run camera_system camera_viewer.py  --ros-args -p output_dir:=$1
}

...

dataset () {
    echo "::: CREATING DATASET :::" in $1 with camera $2
    while true; do
        sleep 15  ; echo "STARTING A NEW ITERATION"
        sim &
        static_agent &
        sleep 10  ; humanoid &
        sleep 5   ; camera $1 &
        sleep 5   ; random $1 &
        sleep 1200; kill
    done
}
```
