import docker.errors

from plugins.plugin import Module
from queue import PriorityQueue


class Trial(Module):
    def __init__(self, trial_id=None):
        super(Trial, self).__init__()
        self.queue = PriorityQueue()
        self.trial_id = trial_id if trial_id is not None else self.uuid

    def add_plugins(self, *plugins):
        for plugin in plugins:
            self.queue.put(plugin)
            self.add(plugin)

    def build(self):
        for plugin in self.children:
            plugin.build()

    def run(self):
        while not self.queue.empty():
            plugin = self.queue.get()
            plugin.run()

    def shutdown(self):
        for plugin in self.children:
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
