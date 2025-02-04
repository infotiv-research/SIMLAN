from simlan_custom_msg.action import TeleportRobot
from scenario_execution.actions.base_action import BaseAction
import py_trees


class CustomAction2(BaseAction):
    def __init__(self):
        super().__init__()

    def execute(self, data: str):
        self.data = data

    def update(self):
        print(f"Custom Action 2 Triggered. Data: {self.data}")
        return py_trees.common.Status.SUCCESS
