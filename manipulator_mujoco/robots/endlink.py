import os
from dm_control import mjcf
from manipulator_mujoco.robots.gripper import Gripper

xml_path = os.path.join(
    os.path.dirname(__file__),
    '../assets/robots/endlink/model.xml',
)

class ENDLINK(Gripper):
    def __init__(self, name: str = None):
        self._mjcf_root = mjcf.from_path(xml_path)
        if name:
            self._mjcf_root.model = name
        # Find MJCF elements that will be exposed as attributes.
        # self._joint = self._mjcf_root.find('joint', joint_name)
        self._bodies = self.mjcf_model.find_all('body')
        # self._actuator = self._mjcf_root.find('actuator', actuator_name)