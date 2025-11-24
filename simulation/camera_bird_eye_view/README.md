# Bird Eye View package

This package aims to contain projection functionality which originates from [projection.ipynb](camera_utility/projection.ipynb) notebook.

This package features being able to select areas of your choosing and create a bird-eye-view of that area by using available cameras, viewing that area.

Command to run birdeye launch file:

```bash
./control.sh birdeye
```

Screenshots:

![bird eye view](/resources/bird_eye_view.png)
![bird eye view gazebo ](/resources/bird_eye_view_gazebo.png)

The package also includes a camera save node. This node saves raw, depth and semantic segmentation images for all the enabled camera id's in config.sh. To use this, run the following command:
```bash
./control.sh save_depth_seg_images
```
