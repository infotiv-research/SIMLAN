# pallet_truck Description

This packages contains the meshes and URDF of the pallet_truck robot, its supported sensors, and their supported mounts.
At the moment there are 2 viable pallet_truck urdfs. 

- pallet_truck.urdf.xacro
- pallet_truck_simpkle_collisions.urdf.xacro

The puropse is to make them the same but with the simple collision using a box as a collision tag instead of the mesh itself.

To switch between them update the following code in simulation/pallet_truck/pallet_truck_bringup/launch/gazebo.launch.py:

```python
PathJoinSubstitution(
                [
                    FindPackageShare("pallet_truck_description"),
                    "urdf",
                    "pallet_truck.urdf.xacro", #<----
                ]
            ),
```


## Sensors

- **GPS:** Novatel Smart6 and Smart7
- **2D LiDAR:** SICK LMS1xx
- **2D LiDAR:** Hokuyo UST-10
- **3D LiDAR:** Velodyne VLP16 and HDL-32E
- **Camera:** Flir/Pointgrey Flea3 and Flea3 Stereo
- **Camera:** Flir/Pointgrey Bumbleebee2
- **Camera:** Flir/Pointgrey BlackflyS
