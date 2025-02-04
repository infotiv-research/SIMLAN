from simlan_custom_msg.action import TeleportRobot
from scenario_execution.actions.base_action import BaseAction
import py_trees


class CustomAction(BaseAction):
    def __init__(self):
        super().__init__()

    def execute(self, data: str):
        self.data = data

    def update(self):
        print(f"Custom Action Triggered 1. Data: {self.data}")
        return py_trees.common.Status.SUCCESS


class TeleportRobotAction(CustomAction):
    def __init__(self):
        super().__init__()

    def execute(self, goal_handle):
        self.data = f"Teleport request for robot: {goal_handle.request.robot_id}"
        goal_handle.succeed()
        result = TeleportRobot.Result()
        result.success = True
        result.message = f'No operation performed for {goal_handle.request.robot_id}.'
        return result

    def update(self):
        print(f"Teleport Action Triggered. Data: {self.data}")
        return py_trees.common.Status.SUCCESS
