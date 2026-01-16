## Motion planner

The motion planner package has three features.

- it can execute a infinite series of random motions which is useful when you want to create synthetic data.
- you can send motions to execute directly in memory.
- you can pass a motion file to be read and executed

When the package runs it subscribes to a topic: `humanoid_X/execute_motion` which inputs a stringified message of a dict (joint_name, value) or a filename.

**Requirement**: Running this command require you to run these nodes first:

```
replay_motion_namespace="humanoid_1"

./control.sh sim # run simulation
./control.sh humanoid # spawn humanoid
./control.sh moveit # allow humanoid to run motions
```

### file mode

You can replay each motion data file separately:

```
./control.sh execute_motion simulation/motion_planner/motions/default_motion.json
```

> If you don't have any motions available, there are ready ones inside the `motions/` folder.

### string mode

The easiest way to use it is in python using a dictionary, stringifying it with json, and then publishing it to the topic:

```
motion_dict = {JOINT_DATA}
json_str = json.dumps(motion_dict)
cmd = [
    "ros2",
    "topic",
    "pub",
    "--once",
    f"{humanoid_namespace}/execute_motion",
    "std_msgs/msg/String",
    f"{{data: '{json_str}'}}",
]
subprocess.run(cmd, check=True)
```

### random mode

To send random motion to the humanoid use the command below

```
ros2 launch motion_planner random_motion.launch.py run_random_generate:=true output_dir:=$1 "log_level:=${log_level}"
```

or alternatively:

```
./control.sh random_motion
```
