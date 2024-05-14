from modules.base_module import DockerModule
from core.components import ProjectPath
from docker import DockerClient

DEFAULT_PATH = str(ProjectPath/"dockerfiles/hmrs/BehaviourTrees")


class BehaviourTrees(DockerModule):
    def __init__(self, docker_client: DockerClient, config: str, path: str = DEFAULT_PATH, tag: str = "bt_skills",
                 command: str = None, *args, **kwargs):
        super(BehaviourTrees, self).__init__(docker_client=docker_client, path=path, tag=tag,
                                             command=command, *args, **kwargs)
        if self.command is None:
            self.command = f"""bash -c "source /ros_ws/install/setup.bash && export PYTHONPATH=$PYTHONPATH:/ros_ws/src/py_trees_ros_behaviors/launch\
            && ros2 launch py_trees_ros_behaviors skill_engine.py" """
        self.env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        self.env['CONFIG'] = str(config).replace('\'', '\"')
