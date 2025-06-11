# Pallet Truck navigation

- This package covers the functionality of mapping, localizing, and navigation for the pallet truck.

- Each package is built using code from the nav2 packages, thus you are able to modify the launch files but to change the node's behaviour you need to modify the config files

- The code is tailored for the pallet truck and using the `aruco_detection` package that supplies odometry for the truck.

- Mapping (otherwise known as cartography), you're able to run from the `cartography.launch.py` package. It requires a lidar to be mounted on the pallet truck and odometry to exist, which can be activated from the [camera_utility/aruco_localization package](camera_utility/aruco_localization).

  - To start mapping, run this package, run gazebo, and rviz, and aruco_localization, and start moving around the simulation with the pallet truck. When your finished you need to save it using this command: `ros2 run nav2_map_server map_saver_cli -f simulation/pallet_truck/pallet_truck_navigation/maps/YOUR_MAP_NAME`.

- Localization, you're able to localize the pallet truck in rviz using the `localization.launch.py` package. It require you to have a map file and odometry running use the aruco_localization package.

- Navigation, you're able to set a goal pose for the pallet truck and it will starting moving towards that goal by it self, using the `navigation.launch.py`. Running this package requires you to also run `localization.launch.py` in a parallel terminal and `aruco_localization`.

- There exists three config files where you can modify the setup for each of the three packages, they are found in the `config` folder.

- Maps are stored in the `maps` folder
