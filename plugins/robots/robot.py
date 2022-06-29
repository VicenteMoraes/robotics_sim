from plugins.plugin import DockerPlugin
import docker


class Robot(DockerPlugin):
    def __init__(self, command: str = "", container_name: str = "robot", *args, **kwargs):
        super(Robot, self).__init__(container_name=container_name, command=command, *args, **kwargs)
