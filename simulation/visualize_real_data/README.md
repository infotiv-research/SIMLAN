## Preparing Data

### Add the data

Add your data files beneath the `replay_data` directory. Your structure should look like this:

```bash
replay_data/
├── images/
│   └── images
└── data
```

By default, the package expects to find the trajectory data inside the `replay_data/` folder as `.json`, and the images inside the `images/` folder as `.jpg`. The .json trajectories are expected on this shape:

```json
{
  "time_stamp": 1746009292618, // Unix timestamp in milliseconds
  "object_list": [
    [
      0, // Tracking ID
      1, // X
      2, // Y
      3 // Z (most often 0)
    ]
  ]
}
```

Notes:

- The `replay_data` folder is gitignored, so you can safely save images there without uploading to git.

- The **folder-names are configurable**, as long as the settings in `params.yaml` match.

- You can:

  - Rename the `images/` folder to something else.
  - Rename the trajectory JSON file to any other valid filename.

- Just be sure to update the corresponding fields (`images_folder`, `json_file_name`) in `params.yaml`.

### Build the package

For example with:

```bash
colcon build --merge-install --symlink-install --packages-select visualize_real_data
```

______________________________________________________________________

## Config file

When your data is in place, you may want to review the config parameters in `params.yaml`. This step is often optional, as default settings typically work well.

### `params.yaml` Overview

#### `prepare_data` section:

- The Parameters here have to be set before `prepare.launch.py` is run. Changes made afterwards will not affect the preparation process (image->PointCloud2).

| Parameter             | Description                                                                                                     | Default Value      |
| --------------------- | --------------------------------------------------------------------------------------------------------------- | ------------------ |
| `pointcloud_topic`    | Topic to publish the `PointCloud2` (image data).                                                                | `pointcloud_topic` |
| `images_folder`       | Name of the folder containing images.                                                                           | `images`           |
| `image_scale`         | Resize factor for `.jpg` images (useful for RViz display).                                                      | `0.04`             |
| `set_frames`          | If `true`, limits processing to `frames_to_process`; otherwise, all available images will be used.              | `false`            |
| `frames_to_process`   | Number of frames to process if `set_frames` is `true`.                                                          | `1`                |
| `preprocess_all_data` | If `true`, preprocesses all available data before playback. May not work for extremely large recording windows. | `true`             |
| `fake_orientation`    | If `true`, automatically fakes object orientations during data preparation (used by the scenario replayer).     | `true`             |

#### `send_data` section:

- Parameters here can be changed after the prepare-stage to alter the playback.
  *You do not need to rebuild the package or re-run prepare.launch.py* if you change something here.

| Parameter        | Description                                                                   | Default Value               |
| ---------------- | ----------------------------------------------------------------------------- | --------------------------- |
| `playback_rate`  | Controls playback speed. Default is `1` (real-time based on `extracted_fps`). | `1`                         |
| `frame_position` | `x`,`y`,`z` coordinates for visualization frame.                              | `x: 15.35, y: 5.7, z: -0.2` |
| `bag_name`       | Name of the rosbag to play. If empty, the most recent one is used.            | *(empty)*                   |

#### `scenario_replayer` section:

| Parameter                 | Description                                                                                                                                                                                     | Default Value             |
| ------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------- |
| `gazebo_teleport_service` | Service name for teleporting robots in Gazebo.                                                                                                                                                  | `/world/default/set_pose` |
| `frame_id`                | Frame in which entities are visualized.                                                                                                                                                         | `real_data`               |
| `use_cmd_vel`             | If `true`, robots are driven with velocity commands; otherwise, teleportation is used to reenact the scenario. When set to `true`, it is important the simulation runs close to 100% real-time. | `false`                   |
| `ignore_orientation`      | If `true`, robot orientation is reset to identity (no rotation) regardless of source data. Using with `cmd_vel` may cause unexpected movement behavior.                                         | `false`                   |

#### `shared` section:

- Parameters here have to be set at the prepare-stage and can't be changed afterwards.

| Parameter               | Description                                                                         | Default Value         |
| ----------------------- | ----------------------------------------------------------------------------------- | --------------------- |
| `frame_id`              | Name of the frame in which all data is displayed.                                   | `real_data`           |
| `namespace`             | Namespace used for the node.                                                        | `visualize_real_data` |
| `entity_topic`          | Topic to publish the `MarkerArray` (trajectory data).                               | `entity_topic`        |
| `json_file_name`        | Name of the file containing trajectory data.                                        | (empty)               |
| `extracted_fps`         | FPS of the extracted data for playback.                                             | `10.0`                |
| `processing_time_limit` | Max time allowed per frame for consistent playback. If exceeded, a warning appears. | `0.8`                 |

______________________________________________________________________

### Launching the Processing Step

Once configured, first run:

```bash
./control.sh replay_sim
```

This starts the simulation with the correct rviz config.

The run either:

```bash
ros2 launch visualize_real_data prepare.launch.py
```

or

```bash
./control.sh prepare
```

This will first start the `orientation_faker` node, which adds an orientation to the data based on the angle between points in the trajectory (this step can be ignored by changing the `fake_orientation` parameter). Afterwards, the data processing is started and generates a rosbag file stored in a `rosbags/` folder inside the `replay_data` directory.

> __NOTE:__ If orientation is already given, change the `fake_orientation` parameter inside the config to prevent it from overwriting the data.

- Rosbags are named using the name of the JSON file used and timestamps (e.g., `my_recording_20250722_153045` if the JSON file is named `my_recording.json`).
- Existing rosbags **are never overwritten**.
- You can delete old rosbags manually from the `rosbags/` folder if needed.

## Sending Data

After processing the data, you can send it to RViz:

1. Make sure you’ve added the following displays to RViz:

   - `PointCloud2`
   - `MarkerArray`

1. Run:

```bash
ros2 launch visualize_real_data send.launch.py
```

or

```bash
./control.sh replay_rviz
```

This command sends the most **recently created rosbag** to the topics defined in `params.yaml`.

- To play an earlier dataset, you must manually delete newer rosbags as the latest one (by timestamp) is always selected automatically.

### Adjusting Playback Speed

You can modify the `playback_rate` parameter in `params.yaml` to control how quickly the rosbag is replayed.

- A value of `1` reflects real-time playback based on extracted timestamps.
- Slower or faster playback is supported, but *extreme values haven't been tested*.

### Fixing RViz

- Usually the displays in RViz have to be configured to subscribe to the correct topics, and the PointCloud2 display's size has to match the image_scale defined in the `params.yaml` config file to look right.

- You can also change `Alpha` in the PointCloud display to see through the image.

- If the images aren't showing up, sometimes the `QOS (Quality of Service)` settings might be mismatched between the `rosbag player` and `RViz`.
  The intended `QOS settings` can be found in the `recorder_qos.yaml` file in the `config` folder together with the `params.yaml` file.
  These are the settings that the rosbag player uses, and the easiest fix is to make sure that RViz mirrors these settings for the respective displays.

______________________________________________________________________

## Summary

1. Build package using `CTRL + SHIFT + B` VS Code task.
1. Add the data you want to show in RViz in the `replay_data` folder
1. Configure parameters under 'prepare_data', 'shared', and/or 'scenario_replayer' *(optional)*
1. Run `prepare.launch.py`
1. Open RViz2 and add the displays `PointCloud2` and `MarkerArray`
   - Make sure the QOS-settings match between the rosbag player and the displays
1. Configure parameters under 'send_data' *(optional)*
1. Run `send.launch.py`
1. Subscribe to the relevant topics (as defined in `params.yaml`) in RViz2

Now the data should be visualized.

______________________________________________________________________

### Warnings and Errors from the node

Warnings and Errors thrown by this package are intended to inform you when something unexpected occurs during execution. In most cases, the process can continue without interruption, although the output may be affected.

It is strongly recommended to resolve the underlying issue based on the warning/error message and then **re-run the process** to ensure reliable and consistent output.
