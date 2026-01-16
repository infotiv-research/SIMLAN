### Integration tests

To run the tests in any package you need to run: `colcon test` or for specific packages: `colcon test --packages-select integration_tests`

Here is an example of a more complex command you can run: `colcon test --packages-select integration_tests --event-handlers console_direct+ --merge-install --pytest-args "-s"`.

- `--event-handlers console_direct+`: prints output to console
- `--merge-install`: Use a merged install space instead of one per package.
- `--pytest-args "-s"`: Pass -s to pytest and do not capture stdout/stderr, so print() shows up.

#### Adding tests.

If you want to add your own tests, the most important part is to prefix the filename with "test\_" in order for colcon to find it.

A template for a test file can look like this below containing a fixture/launch_description and a test case:

```bash

### Fixture
@launch_pytest.fixture(autouse=True)
def launch_description():

    ######## Arguments ########
    world_setup = "default"
    log_level = "error"
    humanoid_str = '[{"namespace": "humanoid_1","initial_pose_x":10,"initial_pose_y":0.0}]'

    ######## Launch-files / Processes  ########
    os.environ["ROS_DOMAIN_ID"] = str(os.getpid() % 232)  #

    sim_proc = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "simlan_bringup",
            "sim.launch.py",
            f"log_level:={log_level}",
            f"world_setup:={world_setup}",
            f"headless_gazebo:=true",
        ],
        output="screen",
    )
    humanoid_proc = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "humanoid_robot",
            "multiple_humanoid_spawn.launch.py",
            f"log_level:={log_level}",
            f"humanoids:={humanoid_str}",
        ],
        output="screen",
    )

    ######## Test execution ########
    yield launch.LaunchDescription(
        [
            sim_proc,
            humanoid_proc,
            launch_pytest.actions.ReadyToTest(),
        ]
    )

### TEST
@pytest.mark.launch(fixture=launch_description)
async def test_sim_and_multiple_robots_bringup_Startup_Nodes_and_topics_should_be_visible():
    print(
        "STARTING TEST: sim_and_multiple_robots_bringup_Startup_Nodes_and_topics_should_be_visible "
    )
```

#### Information

This package aims to test all the features in the project with the use of `Launch_pytest` as the testing framework.

To view the implemented tests, go to the `test/` directory. All tests need to be named with the prefix test\_ for colcon to run them. Each test file should only cover one area and also, each test should only cover one feature within the test. Meaning a test file might cover humanoid, and one of the tests, checks if it can start normally.

A helper node is used that acts as the communicator to the ros2 interface. Its primarily used to fetch the topics and nodes that exist. This helper node is placed [here](./test/test_node.py).

The structure within each file is the following:

#### Fixture

The test fixture acts as the setup for each test, where repeated logic exists. Before every test is ran, the fixture runs, and when the test is finished, the fixture shuts down.

#### Test method

The tests have been written to follow the guideline from [osherove.com](https://osherove.com/blog/2005/4/3/naming-standards-for-unit-tests.html) where each test covers one thing only and its naming convention is "UnitOfWork_StateUnderTest_ExpectedBehavior".
