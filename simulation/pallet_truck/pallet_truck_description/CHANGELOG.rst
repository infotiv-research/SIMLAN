^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pallet_truck_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2023-04-20)
------------------
* [pallet_truck_description] Changed the output type of the gazebo_ros_ray_sensor to LaserScan.
* Contributors: Tony Baltovski

1.0.3 (2023-02-08)
------------------
* Fixed gazebo_ros_laser plugin
* Contributors: Roni Kreinin

1.0.2 (2023-02-02)
------------------
* Add missing dependency to velodyne_description.
* Contributors: Joey Yang

1.0.1 (2022-10-14)
------------------

1.0.0 (2022-09-27)
------------------
* Updated all packages to 0.8.5.
* Added sim support
* Added Nav2 and slam_toolbox to pallet_truck_navigation
* Control fixes
* Minor cleanup
* Added ros2_control to description
* Remove RViz Dependency
* Hack until lms1xx and pointgrey_camera_description are available
* ROS 2 Port
* Contributors: David V. Lu, Roni Kreinin, Tony Baltovski

0.8.1 (2022-01-18)
------------------
* Updated to match melodic-devel
* Contributors: Luis Camero

0.8.0 (2021-04-23)
------------------

0.7.5 (2021-03-24)
------------------
* Add the origin block to the fender UST-10 macros; otherwise enabling them crashes
* Contributors: Chris I-B

0.7.4 (2021-03-16)
------------------
* Bumped CMake version to avoid author warning.
* Contributors: Tony Baltovski

0.7.3 (2021-03-08)
------------------
*  Add VLP16 support, refactor main/secondary laser envar support (#79)
* Contributors: Chris I-B

0.7.2 (2020-09-29)
------------------

0.7.1 (2020-08-24)
------------------

0.7.0 (2020-04-20)
------------------
* [pallet_truck_description] Re-added pointgrey_camera_description as run depend.
* Contributors: Tony Baltovski

0.6.4 (2020-03-04)
------------------
* Modify the hokuyo accessory so that it works properly in gazebo/rviz.  Add an additional environment var pallet_truck_LASER_HOKUYO which overrides the default lms1xx sensor with the ust10.
* use env_run.bat on Windows (`#3 <https://github.com/pallet_truck/pallet_truck/issues/3>`_)
* add setlocal
* Fix pallet_truck_description install location & fold xacro includes (`#2 <https://github.com/pallet_truck/pallet_truck/issues/2>`_)
  * Fix install location.
  * Fold xacro includes
* add env-hook batch scripts (`#1 <https://github.com/pallet_truck/pallet_truck/issues/1>`_)
* Contributors: Chris I-B, James Xu, Sean Yen, Tony Baltovski

0.6.3 (2019-07-18)
------------------
* Added all extra fender changes
* Contributors: Dave Niewinski

* Made minor changes to syntax for kinetic warnings
* Contributors: Dave Niewinski

* Added stereo camera accessory.
* Removed unused variable pallet_truck_description_dir
* Make urdf refer explicitly to pallet_truck_description, rather than relying on current working directory being correct, for easier external includes
* Contributors: Arnold Kalmbach, Tony Baltovski, akalmbach

0.5.1 (2015-02-02)
------------------
* Modified the accessories.urdf.xacro to include both the GPS and mount plate, including standoffs.
* Eliminate rosrun from the xacro wrapper.
* Contributors: BryceVoort, Mike Purvis

0.5.0 (2015-01-20)
------------------
* Add hook for custom URDF insertion to pallet_truck.urdf.xacro.
* Add xacro wrapper script to provide some pre-cooked "configs", especially for simulated pallet_truck.
* Switch to parameterizing URDF with optenv.
* Add laser bracket STL.
* Contributors: Mike Purvis

0.4.2 (2015-01-14)
------------------

0.4.1 (2015-01-07)
------------------

0.4.0 (2014-12-12)
------------------
* add pointgrey camera
* Removed inertial and geometry of the base_link.
* hector gazebo plugin for gps is added.
* hector gazebo plugin for imu sensor is added
* Contributors: Mike Purvis, spourmehr

0.3.0 (2014-09-10)
------------------
* Add comment about accessory args.
* Add front laser accessory to description.
* Contributors: Mike Purvis

0.2.1 (2014-09-10)
------------------

0.2.0 (2014-09-09)
------------------
* Changed physical and collision properties.
* Fixed inertia parameters. Added imu plugin--not working
* Install launch directory.
* Contributors: Mike Purvis, Shokoofeh

0.1.1 (2014-09-06)
------------------
* Remove unnecessary find packages.
* Contributors: Mike Purvis

0.1.0 (2014-09-05)
------------------
* Updated description with v0.9 hardware changes.
* Contributors: Mike Purvis
