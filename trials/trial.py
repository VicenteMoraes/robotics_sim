import docker.errors

from core.components import Component
from queue import PriorityQueue


class Trial(Component):
    def __init__(self, trial_id=None):
        super(Trial, self).__init__()
        self.queue = PriorityQueue()
        self.plugins = []
        self.trial_id = trial_id if trial_id is not None else self.uuid

    def add_plugins(self, *plugins):
        for plugin in plugins:
            self.queue.put(plugin)
            self.plugins.append(plugin)
            self._add(plugin)

    def build(self):
        for plugin in sorted(self.plugins):
            try:
                plugin.build()
            except docker.errors.BuildError as error:
                print(f"\n\n FAILED TO BUILD {plugin.__class__.__name__}\n\n")
                raise error

    def run(self):
        while not self.queue.empty():
            plugin = self.queue.get()
            plugin.run()

    def shutdown(self):
        for plugin in self.plugins:
            plugin.stop()
        try:
            self.event_callback(msg="RUN NEXT TRIAL")
        except AttributeError:
            pass

    def event_callback(self, msg: str):
        match msg:
            case "SHUTDOWN":
                self.shutdown()
            case _:
                super(Trial, self).event_callback(msg)
