from modules.robots.robot import Robot
from docker import DockerClient
from core.components import ProjectPath

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Robots/Human")


class Human(Robot):
    def __init__(self, docker_client: DockerClient, path: str = DEFAULT_PATH, tag: str = "human", robot_name: str = "human",
                 command: str = "", *args, **kwargs):
        super(Human, self).__init__(docker_client=docker_client, path=path, tag=tag, robot_name=robot_name, command=command,
                                    *args, **kwargs)

        self.command = f"""bash -c "python3 /workdir/launch/robot_bringup.py {robot_name} {robot_name} {str(self.initial_pose)} \
        && python3 /workdir/launch/nurse.py" """
        self.add_mount(source=DEFAULT_PATH+"/launch", target="/workdir/launch")
