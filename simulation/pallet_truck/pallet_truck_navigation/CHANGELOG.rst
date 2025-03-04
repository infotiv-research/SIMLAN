^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pallet_truck_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.5 (2022-05-17)
------------------

0.8.4 (2022-05-09)
------------------

0.8.3 (2022-03-08)
------------------

0.8.2 (2022-02-15)
------------------

1.0.4 (2023-04-20)
------------------

1.0.3 (2023-02-08)
------------------

1.0.2 (2023-02-02)
------------------

1.0.1 (2022-10-14)
------------------

1.0.0 (2022-09-27)
------------------
* Use Substitions for launch files
  Added back CHANGELOG.rst
* Linter fixes
* Fixed imu filter node name
  Updated scan topic to use /front/scan by default
* Added Nav2 and slam_toolbox to pallet_truck_navigation
* ROS 2 Port
* Contributors: David V. Lu, Roni Kreinin

0.8.1 (2022-01-18)
------------------

0.8.0 (2021-04-23)
------------------

0.7.5 (2021-03-24)
------------------

0.7.4 (2021-03-16)
------------------
* Bumped CMake version to avoid author warning.
* Contributors: Tony Baltovski

0.7.3 (2021-03-08)
------------------

0.7.2 (2020-09-29)
------------------
* Use the pallet_truck_LASER_TOPIC env var as the default for the scan topic in amcl + gmapping demos (`#74 <https://github.com/pallet_truck/pallet_truck/issues/74>`_)
* Contributors: Chris I-B

0.7.1 (2020-08-24)
------------------
* Remove the leading / from the gmapping default scan topic
* Expose the scan_topic and use_map_topic parameters in the demo launch files
* Contributors: Chris Iverach-Brereton

0.7.0 (2020-04-20)
------------------

0.6.4 (2020-03-04)
------------------

0.6.3 (2019-07-18)
------------------

0.6.2 (2019-04-18)
------------------

0.6.1 (2018-08-02)
------------------

0.6.0 (2018-04-12)
------------------

0.5.4 (2018-04-12)
------------------

0.5.3 (2016-06-01)
------------------

0.5.1 (2015-02-02)
------------------

0.5.0 (2015-01-20)
------------------

0.4.2 (2015-01-14)
------------------

0.4.1 (2015-01-07)
------------------

0.4.0 (2014-12-12)
------------------
* Removed redundant local and global costmaps.
* Fixed location of non-demo launch files in roslaunch-file-check in CMakeList
* Moved rviz configurations to pallet_truck_viz.
* Changed config/ to params/. Moved non-demo launch files to launch/include.
* Added gps_transform.launch for transforming GPS data to robot's world frame. Also added gps_integration_demo.launch for fusing GPS data with odomtery data. This node generates map->odom transform.
* Modified AMCL's parameters, map and demo for pallet_truck.
* Modified slam_gmapping parameters for pallet_truck for building a map.
* Modified costmap and base planner parameters for the navigation without map with obstacle avoidance using laser scanner.
* Contributors: Mike Purvis, Shokoofeh Pourmehr

0.3.0 (2014-09-10 16:25)
------------------------

0.2.1 (2014-09-10 08:54)
------------------------

0.2.0 (2014-09-09)
------------------

0.1.1 (2014-09-06)
------------------

0.1.0 (2014-09-05)
------------------