## Preparing Data

### First build the package

for example with:

```bash
colcon build --merge-install --symlink-install --packages-select visualize_real_data
```

After that, you can add the required data into the `share/` folder that is created during the build process.

If everything has been built correctly you should find the following folder-structure:

```bash
install/
└── share/
    └── visualize_real_data/
        └── data/
            ├── images/
            │   └── images
            └── data
```

The empty `data` and `images` placeholder files ensure that ROS 2 recognizes and preserves the folder structure during the build
and should **not** be removed.

If you have already built the package before there might be another directory inside the `data/`  folder called `rosbags`.
More on that folder later.

### Add the data after building

After the build completes, you can add your data files to the generated `share/` folder inside your package.

By default the package expects to find the trajectory data inside the `data/` folder as `.json`
and the images inside the `images/` folder as `.jpg`. Like this:

```bash
install/
└── share/
    └── visualize_real_data/
        └── data/
            ├── images/
            │   ├── image1.jpg
            │   └── image2.jpg
            └── window_2012_1746008869618_1746008890118.json
```

Notes:

* The **folder-names are configurable**, as long as the settings in `params.yaml` match.
* You can:

  * Rename the `images/` folder to something else.
  * Rename the trajectory JSON file to any other valid filename.

* Just be sure to update the corresponding fields (`images_folder`, `json_file_name`) in `params.yaml`.

---

### Important Notes

* You do *not* need to rebuild the package after adding or modifying data in `share/`.
* In fact, **rebuilding the package could overwrite or remove any manually added files** in the `share/` folder,
  so avoid rebuilding once your data is in place.

## Config file

When your data is in place, you may want to review the config parameters in `params.yaml`. This step is often optional, as default settings typically work well.

### `params.yaml` Overview

#### `prepare_data` section:

* The Parameters here have to be set before `prepare.launch.py` is run. Changes made afterwards will not
  effect the preparation process (image->PointCloud2).

| Parameter           | Description                                                                                        | Default Value        |
| ------------------- | -------------------------------------------------------------------------------------------------- | -------------------- |
| `pointcloud_topic`  | Topic to publish the `PointCloud2` (image data).                                                   | `pointcloud_topic`   |
| `images_folder`     | Name of the folder containing images.                                                              | `images`             |
| `image_scale`       | Resize factor for `.jpg` images (useful for RViz display).                                         | `0.04`               |
| `set_frames`        | If `true`, limits processing to `frames_to_process`; otherwise, all available images will be used. | `false`              |
| `frames_to_process` | Number of frames to process if `set_frames` is `true`.                                             | `1`                  |
| `preprocess_all_data` | If `true`, preprocesses all available data before playback. May not work for extremely large recording windows.                                      | `true`               |
| `fake_orientation`   | If `true`, automatically fakes object orientations during data preparation (used by the scenario replayer).                        | `true`               |

#### `send_data` section:

* Parameters here can be changed after the prepare-stage to alter the playback.
  *You do not need to rebuild the package or re-run prepare.launch.py* if you change something here.

| Parameter        | Description                                                                   | Default Value |
| ---------------- | ----------------------------------------------------------------------------- | ------------- |
| `playback_rate`  | Controls playback speed. Default is `1` (real-time based on `extracted_fps`). | `1`           |
| `frame_position` | `x`,`y`,`z` coordinates for visualization frame.                              | `x: 15.35, y: 6.5, z: -0.2` |
| `bag_name`       | Name of the rosbag to play. If empty, the most recent one is used.            | *(empty)*     |


#### `scenario_replayer` section:

| Parameter                | Description                                                      | Default Value                |
| ------------------------ | ---------------------------------------------------------------- | ---------------------------- |
| `gazebo_teleport_service`| Service name for teleporting robots in Gazebo.                   | `/world/default/set_pose`    |
| `frame_id`               | Frame in which entities are visualized.                          | `real_data`                  |
| `use_cmd_vel`            | If `true`, robots are driven with velocity commands; otherwise, teleportation is used to reenact the scenario. When set to `true` it is important the simulation runs close to 100% real-time. | `true`                       |


#### `shared` section:

* Parameters here have to be set at the prepare-stage and can't be changed afterwards.

| Parameter               | Description                                                                                        | Default Value |
| ----------------------- | -------------------------------------------------------------------------------------------------- | ------------- |
| `frame_id`              | Name of the frame in which all data is displayed.                                                  | `real_data`   |
| `namespace`             | Namespace used for the node.                                                                       | `visualize_real_data` |
| `entity_topic`          | Topic to publish the `MarkerArray` (trajectory data).                                              | `entity_topic` |
| `json_file_name`        | Name of the file containing trajectory data.                                                       | (empty) |
| `extracted_fps`         | FPS of the extracted data for playback.                                                            | `10.0`        |
| `processing_time_limit` | Max time allowed per frame for consistent playback. If exceeded, a warning appears.                | `0.8`         |

---

### Launching the Processing Step

Once configured, run:

```bash
ros2 launch visualize_real_data prepare.launch.py
```

This will first start the `orientation_fixer` node which adds an orientation to the data based on the angle between points in the trajectory. Afterwards the data processing is started and generates a rosbag file stored in a `rosbags/` folder inside the package's `share/` directory.
> __NOTE:__ If orientation is already given, change 

* Rosbags are named using the name of the JSON file used and timestamps (e.g., `my_recording_20250722_153045` if the JSON file is named `my_recording.json`).
* Existing rosbags **are never overwritten**.
* You can delete old rosbags manually from the `rosbags/` folder if needed.

## Sending Data

After processing the data you can send it to RViz:

1. Make sure you’ve added the following displays to RViz:

   * `PointCloud2`
   * `MarkerArray`

2. Run:

```bash
ros2 launch visualize_real_data send.launch.py
```

This command sends the most **recently created rosbag** to the topics defined in `params.yaml`.

* To play an earlier dataset, you must manually delete newer rosbags as the latest one (by timestamp) is always selected automatically.

### Adjusting Playback Speed

You can modify the `playback_rate` parameter in `params.yaml` to control how quickly the rosbag is replayed.

* A value of `1` reflects real-time playback based on extracted timestamps.
* Slower or faster playback is supported, but *extreme values haven't been tested*.

### Fixing RViz

* Usually the displays in RViz have to be configured to subscribe to the correct topics,
  and the PointCloud2 displays size have to match the image_scale defined in the `params.yaml` config file to look right.
* You can also change `Alpha` in the PointCloud display to see through the image.

* If the images aren't showing up, sometimes the `QOS (Quality of Service)` settings might be mismatched between the `rosbag player` and `RViz`.
  The intended `QOS settings` can be found in the `recorder_qos.yaml` file in the `config` folder together with the `params.yaml` file.
  These are the settings that the rosbag player uses, and the easiest fix is to make sure that RViz mirrors these settings for the respective displays.

---

## Summary

1. Build package using `CTRL + SHIFT + B` VS Code task.
2. Add the data you want to show in RViz in the package's `share/` folder
3. Configure parameters under 'prepare_data', 'shared', and/or 'scenario_replayer' *(optional)*
4. Run `prepare.launch.py`
5. Open RViz2 and add the displays `PointCloud2` and `MarkerArray`
   - Make sure the QOS-settings match between the rosbag player and the displays
6. Configure parameters under 'send_data' *(optional)*
7. Run `send.launch.py`
8. Subscribe to the relevant topics (as defined in `params.yaml`) in RViz2

Now the data should be visualized.

---

### Warnings and Errors from the node

Warnings and Errors thrown by this package are intended to inform you when something unexpected occurs during execution. In most cases, the process can continue without interruption, although the output may be affected.

It is strongly recommended to resolve the underlying issue based on the warning/error message and then **re-run the process** to ensure reliable and consistent output.
