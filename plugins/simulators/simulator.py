from plugins.plugin import DockerPlugin

class Simulator(DockerPlugin):
    def __init__(self, headless_command: str, gui_command: str, headless: bool = True, tag: str = "sim",
                 path_to_world: str = "", world_arg: str = "world_map", *args, **kwargs):
        super(Simulator, self).__init__(tag=tag, command="", *args, **kwargs)
        self.headless = headless

        if not self.headless:
            self.command = gui_command
            self.load_gui()
        else:
            self.command = headless_command

        if path_to_world:
            self.command += ' ' + f"{world_arg}:={path_to_world}"
