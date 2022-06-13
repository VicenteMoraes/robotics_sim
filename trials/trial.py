from core.components import Component
from queue import PriorityQueue


class Trial(Component):
    def __init__(self, trial_id=None):
        super(Trial, self).__init__()
        self.queue = PriorityQueue()
        self.plugins = []
        self.trial_id = trial_id if trial_id is not None else self.uuid

    def add_plugin(self, plugin):
        self.queue.put(plugin)
        self.plugins.append(plugin)
        self._add(plugin)

    def build(self):
        for plugin in self.plugins:
            plugin.build()

    def run(self):
        while not self.queue.empty():
            plugin = self.queue.get()
            plugin.run()
