# SIMLAN bringup

This package is responsible for launching all things related to the simulation. The launch files in the package calls launch files in all other packages to start everything up with the following command

`ros2 launch simlan_bringup full_sim.launch.py`

This is also called in the `sim` operation in control.sh:
```bash
./control.sh sim
```

## Launch arguments

There are different launch arguments that can be changed to launch the simulation in different states. These can either be called from the terminal by appending the argument to the launch command above.

`rviz:='True'`

Another way to edit what is launched it to change the _default value_ in [_full_sim.launch.py_](launch/full_sim.launch.py).

`launch_rviz_launch_argument = DeclareLaunchArgument("rviz", default_value="False", description="To launch rviz")`

The launch arguments are then either added as a condition straight to a Node:

`condition=IfCondition(rviz)`

or passed downward to the launch file being called from the top level launch file:

`launch_arguments={"jackal_manual_control":jackal_manual_control,}.items()`

If you use a specific argument configuration often it is best to create a new launch file, on top of [_full_sim.launch.py_](launch/full_sim.launch.py). It can have the correct default values for all arguments and then just call on [_full_sim.launch.py_](launch/full_sim.launch.py).

## Diagram of launch structure

This diagram is made in DrawIO and the png contains the xml code. Drop it into [drawio](https://app.diagrams.net/) to make changes and add back to this readme.
![launch_structure](images/SMILE_IV_Jackal-Simlan_bringup%20launch.drawio.png)
