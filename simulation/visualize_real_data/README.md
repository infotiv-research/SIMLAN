
## Preparing Data

To prepare data for use in this package, you should **first build the package**.

After that, you can add the required data into the `share/` folder that is created during the build process.

---

### Step 1: **Build the Package**

Before working with any data, ensure that the package has been built at least once, *for example* with:

```bash
colcon build --merge-install --symlink-install --packages-select visualize_real_data
```

This step sets up the necessary directory structure, including the `share/` folder where all runtime data will be placed.

---

### Step 2: **Add Data After Building**

After the build completes, you can add your data files to the generated `share/` folder inside your package. For example:

```bash
install/
└── share/
    └── visualize_real_data/
        └── data/
            ├── images/
            │   ├── image1.jpg
            │   └── image2.jpg
            └── trajectories.json
```

Notes:

* The **names in the data folder are fully configurable**, as long as the settings in `params.yaml` match.
* You are free to:

  * Rename the `images/` folder to something else.
  * Rename `trajectories.json` to any other valid filename.

* Just be sure to update the corresponding fields (`images_folder`, `json_file_name`) in `params.yaml`.

---

### Important Notes

* You do **not** need to rebuild the package after adding or modifying data in `share/`.
* In fact, **rebuilding the package could overwrite or remove any manually added files** in the `share/` folder, so avoid rebuilding once your data is in place.
* The empty files `data` and `images` are there to ensure the correct directory structure at build-time and should **not** be removed. 


---

## Config file

When your data is in place, you may want to review the config parameters in `params.yaml`. This step is often optional, as default settings typically work well.

### `params.yaml` Overview

#### `prepare_data` section:

*Parameters here have to be set before `prepare.launch.py` is run and can't be changed afterwards.*

| Parameter           | Description                                                                                        |
| ------------------- | -------------------------------------------------------------------------------------------------- |
| `entity_topic`      | Topic to publish the `MarkerArray` (trajectory data).                                              |
| `frames_to_process` | Number of frames to process if `set_frames` is `true`.                                             |
| `image_scale`       | Resize factor for `.jpg` images (useful for RViz display).                                         |
| `images_folder`     | Name of the folder containing images.                                                              |
| `json_file_name`    | Name of the file containing trajectory data.                                                       |
| `pointcloud_topic`  | Topic to publish the `PointCloud2` (image data).                                                   |
| `set_frames`        | If `true`, limits processing to `frames_to_process`; otherwise, all available images will be used. |

#### `send_data` section:

*Parameters here can be changed after the prepare-stage to alter the playback.*

| Parameter        | Description                                                                   |
| ---------------- | ----------------------------------------------------------------------------- |
| `extracted_fps`  | Auto-calculated based on timestamps; can be set manually if needed.           |
| `image_position` | `x`,`y`,`z` coordinates for visualization frame.                              |
| `playback_rate`  | Controls playback speed. Default is `1` (real-time based on `extracted_fps`). |

#### `shared` section:

*Parameters here have to be set at the prepare-stage and can't be changed afterwards.*

| Parameter               | Description                                                                                        |
| ----------------------- | -------------------------------------------------------------------------------------------------- |
| `frame_id`              | Name of the frame in which all data is displayed.                                                  |
| `processing_time_limit` | Max time allowed per frame for consistent playback. If exceeded, a warning appears.                |
|                         | Playback continues but may be inconsistent. Restarting the process is recommended for consistency. |

---

### Launching the Processing Step

Once configured, run:

```bash
ros2 launch visualize_real_data prepare.launch.py
```

This will start the data processing and generate a rosbag file stored in a `rosbags/` folder inside the package's `share/` directory.

* Rosbags are named using timestamps (e.g., `rosbag_20250722_153045`).
* Existing rosbags **are never overwritten**.
* You can delete old rosbags manually from the `rosbags/` folder if needed.

---

## Sending Data

After processing, you're ready to send the data to RViz.

### Steps:

1. Make sure you’ve added the following displays to RViz:

   * `PointCloud2`
   * `MarkerArray`

2. Run:

```bash
ros2 launch visualize_real_data send.launch.py
```

This command sends the most **recently created rosbag** to the topics defined in `params.yaml`.

* To play an earlier dataset, you must manually delete newer rosbags — the latest one (by timestamp) is always selected automatically.

### Adjusting Playback Speed

You can modify the `playback_rate` parameter in `params.yaml` to control how quickly the rosbag is replayed.

* A value of `1` reflects real-time playback based on extracted timestamps.
* Slower or faster playback is supported, but **extreme values haven't been tested**.

### Fixing Rviz

* Usually the displays in Rviz have to be configured to subscribe to the correct topics, and the PointCloud2 displays size have to match the image_scale defined in the `params.yaml` config file.
* You can also change `Alpha` in the PointCloud display to see through the image.

---

## Summary

1. Build package using `colcon build`
2. Add the data you want to show in Rviz in the package's `share/` folder
3. Configure parameters under 'prepare_data' and/or 'shared' *(optional)*
4. Run `prepare.launch.py`
5. Open Rviz2 and add the displays `PointCloud2` and `MarkerArray`
6. Configure parameters under 'send_data' *(optional)*
7. Run `send.launch.py`
8. Subscribe to the relevant topics (as defined in `params.yaml`) in Rviz2
9. Now the data should be visualized.


---

## Troubleshooting

If you encounter any warnings or error messages during launch, refer to the log output. Most common issues are related to:

* Incorrect folder names in `params.yaml`
* Missing or improperly formatted `.jpg` or trajectory files


### Warnings and Errors from the node

Warnings are intended to inform you when something unexpected occurs during execution. In most cases, the process can continue without interruption, although the output may be affected.

Errors typically do not cause the process to shut down immediately, but they may lead to unpredictable or incorrect results. It is strongly recommended to resolve the underlying issue based on the error message and then **re-run the process** to ensure reliable and consistent output.
