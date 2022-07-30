from plugins.plugin import DockerPlugin
import docker


class Robot(DockerPlugin):
    def __init__(self, command: str, container_name: str = "robot", robot_name: str = "robot", initial_pose=None,
                 *args, **kwargs):
        super(Robot, self).__init__(container_name=container_name, command=command, *args, **kwargs)
        self.robot_name = robot_name
        self.env['ROBOT_NAME'] = robot_name
        self.initial_pose = initial_pose if initial_pose is not None else [0, 0, 0.1]
