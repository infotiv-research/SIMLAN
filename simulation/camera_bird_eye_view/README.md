# Bird Eye View package

As preparation, make sure that the right cameras are selected in the `config.sh`

```bash
## CAMERA_ENABLED_IDS can be set as a string of camera_ids separated by space ' '. valid camera ids are 160-171
CAMERA_ENABLED_IDS='164 165 166 167 168'
CAMERA_STREAMS='image'
```

### Bird eye view

This package features the ability to select areas of your choosing and create a bird-eye view of that area by using available cameras, viewing that area.

Ro run birdeye launch file in separate terminals run the following commands:

```bash
./control.sh build
./control.sh sim
./control.sh static_agents
./control.sh birdeye
```

![bird eye view](/resources/bird_eye_view.png)
![bird eye view gazebo](/resources/bird_eye_view_gazebo.png)

### Different sensors (Depth, Semantic Segmentation, Color)

The package also includes a camera save node. This node saves raw, depth, and semantic segmentation images for all the enabled camera IDs in config.sh. To take a single image from different cameras, run the following command:

```bash
CAMERA_ENABLED_IDS='164 165 166 167 168'
## CAMERA_STREAMS can be set as a string of stream options separated by space ' '. valid camera ids are image depth semantic
CAMERA_STREAMS='image depth semantic'
```

Keep in mind to change `real_time_factor` in [`simulation/simlan_gazebo_environment/worlds/ign_simlan_factory.world.xacro`](/simulation/simlan_gazebo_environment/worlds/ign_simlan_factory.world.xacro) to small values to slow down the simulator (e.g. 0.05-0.1).

to record a camera streams as a sequence of images or a video run the following command in separate terminals:

```bash
./control.sh build
./control.sh sim
./control.sh static_agents
./control.sh save_depth_seg_images
./control.sh save_depth_seg_videos
```

The resulting videos are stored in [`/simulation/camera_bird_eye_view/recordings/`](/simulation/camera_bird_eye_view/recordings/)

![color](/resources/sensors/color.png)

![semantic](/resources/sensors/semantic.png)

![depth](/resources/sensors/depth.png)
