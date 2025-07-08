# Converting `.jpg` Images to `PointCloud2` for RViz2

This package provides a two-step process for converting `.jpg` images into `sensor_msgs/msg/PointCloud2` messages that can be visualized in RViz2. The workflow is executed via the launch file: `prepare.launch.py`.

## Conversion Pipeline

The conversion consists of the following stages:

### 1. Image Sorting

The script `prepare_data.py` organizes images located in:

```
<package_path>/share/realize_pointcloud_images/bev_img/
```

It assumes filenames are based on timestamps or some sortable naming convention.

### 2. Conversion and Publishing

Each image is:

* Converted to a `PointCloud2` message.
* Published to the topic `/realize_pointcloud_images/_internal_rpi_topic` (configurable).
* Recorded by a `rosbag2` recorder into:

```
<package_path>/share/realize_pointcloud_images/cached_images/
```

During execution, logs will display the current progress (e.g., `published 5/20 images`).

> Once all images are processed, press `Ctrl+C` to stop recording. If stopped prematurely, `rosbag2` should gracefully finalize the recording using its internal cache.

---

## Sending `PointCloud2` to RViz2

To visualize the converted data in RViz2, use the launch file: `send.launch.py`.

This launch file:

* Starts a `rosbag2` player.
* Plays back the most recently cached bag file.
* Repeats the playback in a loop.

### Default Playback Parameters

| Parameter             | Default Value         |
| --------------------- | --------------------- |
| `playback_topic_name` | `"pointcloud_images"` |
| `playback_rate`       | `"8"`                 |

> You may need to adjust the playback rate based on the image capture rate or performance during conversion.

To stop playback, press `Ctrl+C` in the terminal.

---

## Summary

* Use `prepare.launch.py` to convert and record `.jpg` images as `PointCloud2`.
* Use `send.launch.py` to visualize the data in RViz2 via rosbag playback.
* Monitor logs for progress and control execution with `Ctrl+C`.
