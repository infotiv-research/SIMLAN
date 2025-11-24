# Integration tests

## Information

This package aims to test all the features in the project with the use of `Launch_pytest` as the testing framework.

To view the implemented tests, go to the `test/` directory. All tests need to be named with the prefix test\_ for colcon to run them. Each test file should only cover one area and also, each test should only cover one feature within the test. Meaning a test file might cover humanoid, and one of the tests, checks if it can start normally.

A helper node is used that acts as the communicator to the ros2 interface. Its primarily used to fetch the topics and nodes that exist. This helper node is placed [here](./test/test_node.py).

The structure within each file is the following:

### Fixture

The test fixture acts as the setup for each test, where repeated logic exists. Before every test is ran, the fixture runs, and when the test is finished, the fixture shuts down.

### Test method

The tests have been written to follow the guideline from [osherove.com](https://osherove.com/blog/2005/4/3/naming-standards-for-unit-tests.html) where each test covers one thing only and its naming convention is "UnitOfWork_StateUnderTest_ExpectedBehavior".
