# Simulation

## World Fidelity

It is possible to adjust the level of fidelity for a world in `config.sh`; there the `world_setup` is sent to

- `simulation/simlan_bringup/launch/sim.launch.py`.
- `simulation/simlan_gazebo_environment/launch/simlan_factory.launch.py` (for world generation)
- `simulation/simlan_gazebo_environment/launch/generate_world_file.py` (both `original_world` and the `world_setup`)

## Adding Aruco and camera (static agents)

Aruco codes and cameras are all attached to the same link in Gazebo. To create new static agents, go to `simulation/static_agent_launcher/description/agents.urdf.xacro`. There you only need to add a new line on the form `<xacro:camera number="1" x="3" y="3" z="6" r="0" p="0" w="0"/>` for a new camera, or a new line on the form `<xacro:aruco number="1" x="3" y="3" z="0.1" r="0" p="0" w="0"/>` for a new aruco. The commands are identical apart from the name.

- `Number` is added to the name to make the static agent unique. For cameras this means they will publish on, e.g., the topic `static_agents/camera_[number]/image_raw`. For aruco the number indicates which aruco png to use.
- `x, y, z` is the coordinate offset from the link the agents are all attached to.
- `r, p, w` are the rotation, pitch, and yaw of the camera, dictating in which direction and at what angle the cameras are looking.

[Gazebo supports simulation of camera](https://classic.gazebosim.org/tutorials?tut=camera_distortion) based on Brown's distortion model. It expects 5 distortion coefficients k1, k2, k3, p1, p2 that you can get from the camera calibration tools. The k coefficients are the radial components of the distortion model, while the p coefficients are the tangential components.

- `<aspect_ratio>`  : The ratio of the width and height of the camera.
- `<horizontal_fov>`: The horizontal field of view of the camera in radians.

[OpenCV](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) uses five parameters, known as distortion coefficients given by like this: `k1, k2, p1, p2 , k3 # pay attention to the order`.

## Using actors in Gazebo

- Placed in sdf or world file
- Actors use the actor tag (as opposed to the model tag most other objects use).
- actor tags contain links like usual, but also a `<script>` tag.
- The script tag contains:
  - loop: Whether the script should loop on completion
  - delay_start: Time to wait before running the script, also waits between loops
  - auto_start: Whether the script should start automatically when the sim starts
  - A trajectory tag, which has an (unique) id and a type (used to couple it with an animation)
    - Inside the trajectory tags we define waypoint tags which consist of a pose and the time where we are supposed to reach the pose.
    - Note: The trajectory is smoothed as a whole. This means that you'll get a fluid motion, but the exact poses contained in the waypoints might not be reached.

Script structure:

```xml
<script>
  <loop>1</loop>
  <auto_start>1</auto_start>
  <trajectory id='0' type='square'>
    <waypoint>
      <time>0</time>
      <pose>-1 -1 1 0 -0 0</pose>
    </waypoint>
    <waypoint>
      <time>1</time>
      <pose>-1 1 1 0 -0 0</pose>
    </waypoint>
    <waypoint>
      <time>2</time>
      <pose>1 1 1 0 -0 0</pose>
    </waypoint>
    <waypoint>
      <time>3</time>
      <pose>1 -1 1 0 -0 0</pose>
    </waypoint>
    <waypoint>
      <time>4</time>
      <pose>-1 -1 1 0 -0 0</pose>
    </waypoint>
  </trajectory>
</script>
```

- Gazebo supports two different skeleton animation file formats: COLLADA (.dae) and Biovision Hierarchy (.bvh).
  - .bvh: Text-based format with section 'HIERARCHY' and section 'MOTION'.
  - .dae: XML-based format, structured into multiple sections.
- .bvh works for Ignition, while .dae works for both Gazebo classic and Ignition
- Skin is similar, simply import a collada file.
- \<interpolate_x>true\</interpolate_x> makes the animation match the movement. The animation is done briefly along the x-axis and then it can be interpolated into any direction by gazebo.
- In a top down positive x,y frame of reference:  0 = right, 1.57 = up, -1.57 = down, 3.14 = left
  - Pos/negative matters! 0 to -1.57 does not behave the same as 0 to 4.71!
  - In other words increasing the angle of rotation means rotating left, and decreasing it means rotating right.
- Tension = how strictly it follows the waypoints in a span 0-1 where 0 is not very strict, 1 very strict

## Simulation Specification

Environment (world)

- An in-door factory warehouse
- An out door street/sidewalk (side walk or street) (FUTURE WORK)

Objects

- human actor
- shelves
- [standard euro pallets](https://en.wikipedia.org/wiki/Pallet)
- [pallet racks](https://www.iqsdirectory.com/articles/storage-rack/pallet-racks.html)
- traffic cone
- Boxes
- [differential-drive AMR (Autonomous Mobile Robot)](https://www.mobile-industrial-robots.com/insights/get-started-with-amrs/agv-vs-amr-whats-the-difference/) (FUTURE WORK)
- forklift (FUTURE WORK)

Textures

- Aruco tag on actor and floor
- objects texture
- Floor texture

Placement of sensors

- camera (depth) on ceiling
- 2D Lidar/camera on AMR (FUTURE WORK)

Physics

- mass
- moment of inertia
- collision
- friction (FUTURE WORK)
- damping

Lighting

Agents movements:

- deterministically following waypoints trajectories (in curve or in form of A-B-C...Z)
- AMR differential-drive using nav2 with obstacle avoidance (FUTURE WORK)
- replaying scenario using rosbag​ (FUTURE WORK)

Additionally, these are expected from the simulator:

- following realistic and standard measurements and sizes for warehouse, pallettes, shelves, cart, human body, etc
- agent/robot status (position) and control via ROS
- exposing and recording camera images and agent status (e.g. positions) in Python
- tele-operation with keyboard and programmatically from python

## AMR

differential-drive (caster wheel)/Ackermann (car) type of robot

`/cmd_vel` topic accepts `Twist` formatted messages; only non-zero value for linear.x (forward/backward) and non-zero value for angular.z (differential drive, rotating around the z axis).

## Warehouse

The warehouse was created in FreeCAD's BIM Workbench. This workbench isn't available by default, but can easily be acquired by going to `Tools` -> `Addon Manager` -> `BIM` -> `Install/update Selected`.

Using the measurements found in the blueprint in the documentation folder, I drew four lines and turned them into walls (line tool and wall tool respectively). By default the wall will be created around the line, so that the line is in the middle of the wall. This can be changed in the wall properties so that it ends up entirely on one side of the line.
As the blueprint lacked walls I used the dimensions specified there as the inner measurements, meaning that the origin point is located at the inner bottom left corner of the wall. All the inner space is in positive x and y coordinates, while the two of the walls are in negative coordinate space.
I used the Aligned Dimension tool from the Annotation tools to confirm that the inner measurements matched those of the blueprint.

While the BIM Workbench has support for door objects, they tend to act as solids when imported into Gazebo, so the door is just a hole in the wall. This hole was created by adding a cube object and having it intersect the wall where the door should be located. Then you select the cube and the wall (in that order) in the tree view and press `Remove Component`. This should remove the cube object and create a hole where the cube and wall overlapped. The hole didn't appear in the right place, but it was possible to edit the position of the hole in its properties. I measured the location of the hole using Aligned Dimension to make sure it ended up in the right spot.

Going to the top view, I created a new rectangle object covering the whole warehouse. Then I turned it into a slab with the slab tool to create a floor for the warehouse.

Everything was added to a level object (note: by default this level object is named "Floor", but don't confuse it with the slab that constitutes the physical floor) so that it's grouped together. If we set the wall heights to 0, they will automatically inherit the height of the Level object, so we can change all the walls easily by changing the height of the level.

I added two materials from presets, concrete and wood. I applied the concrete material to all walls and the wood material to the floor. These should be considered to be temporary placeholders.

Finally, I exported the project as a Collada file (.dae) and added it to a simple .world file to see if it loaded properly in Gazebo, and the results seemed correct.

Since the warehouse will be static, we shouldn't need to define any additional parameters like mass or inertia; visuals and collision should be enough. Textures might need some improvement, as currently they're just basic colors, but it should be possible to add those in FreeCAD and have them included in the .dae file so we can load them visually in Gazebo later.

I also added windows for slightly better visibility.

## Add warehouse to world-file

The floor of the warehouse goes below z=0, so the ground plane was lowered by 0.2 so that the warehouse still rests on top of it. As our simulations will mainly take place inside the warehouse, the warehouse floor replaces the ground plane at z=0. The warehouse itself was placed in the models directory and loaded into the world with an `<include>` tag. Additionally, the maze in the stage4 world was moved away from the origin so that it is fully contained inside the warehouse, though in the future it should be removed altogether.

## Textures

Shelf, pallet, warehouse walls are using CC0 textures from https://polyhaven.com/textures
Box and warehouse floor are using images from Volvo as a base.

- box: picture from [Volvo-group-packaging-specifications_2015.pdf](https://www.volvogroup.com/content/dam/volvo-group/markets/master/suppliers/useful-links-and-documents-for-existing-suppliers/logistics-solutions/volvo-group-packaging-system/Volvo-group-packaging-specifications_2015.pdf)

### Conventions

- Keep your links/joints paired, and use the suffix \_link and \_joint (e.g. arm_link and arm_joint) and maybe follow [REP 120 naming conventions](https://www.ros.org/reps/rep-0120.html).
- Define visual, collision, inertial and Gazebo material (and maybe friction) for all objects

### cartography, localization, navigation

```
ros2 launch pallet_truck_navigation cartography.launch.py
ros2 run nav2_map_server map_saver_cli -f simulation/mapname
```
