from plugins.simulators.simulator import Simulator
from core.components import ProjectPath


class RMFGazebo(Simulator):
    def __init__(self, tag: str = "rmfgazebo", path: str = str(ProjectPath/"dockerfiles/RMFGazebo/"),
                 headless_command="ros2 launch rmf_demos_gz office.launch.xml headless:=1",
                 gui_command='bash -c "xauth add $XAUTH_LIST && ros2 launch rmf_demos_gz office.launch.xml"',
                 *args, **kwargs):
        super(RMFGazebo, self).__init__(headless_command=headless_command, gui_command=gui_command, path=path, tag=tag,
                                        *args, **kwargs)
        self.tag = tag
        self.env["ROS_DOMAIN_ID"] = "9"
        self.headless_command = headless_command
        self.gui_command = gui_command

    def command(self):
        if self.headless:
            return self.headless_command
        return self.gui_command

    def run(self, network_mode: str = "host", **run_kwargs):
        super(RMFGazebo, self).run(network_mode=network_mode, **run_kwargs)

