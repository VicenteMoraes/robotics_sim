from plugins.plugin import Plugin
from plugins.loggers.docker_logger import DockerLogger
from core.components import ProjectPath
from docker import DockerClient
from plugins.networks.ros2_network import ROS2Network


DEFAULT_PATH = str(ProjectPath/"dockerfiles/Ros2/RosLogger")


class ROSLogger(Plugin):
    def __init__(self, docker_client: DockerClient, network: ROS2Network, path: str = DEFAULT_PATH, tag: str = "roslogger",
                 command: str = "python3 /workdir/launch/logger.py", trial_id: int = 0, timeout: float = 15*60,
                 filename: str = "trial_log.log", target: str = "", *args, **kwargs):
        super(ROSLogger, self).__init__(docker_client=docker_client, network=network, path=path, tag=tag,
                                        command=command, *args, **kwargs)
        self.add_logger(write_to_file=True, target=target, filename=filename, timeout=timeout, timeout_stop=True)
        self.env['TRIAL_ID'] = trial_id

        if path == DEFAULT_PATH:
            self.add_mount(source=DEFAULT_PATH+"/launch", target="/workdir/launch")

    def start_logger(self):
        try:
            for child in self.children:
                if type(child) == DockerLogger:
                    child.start_timer()
        except AttributeError:
            pass
