import uuid
from abc import ABC
from pathlib import Path

ProjectPath = Path(__file__).parent.parent


class Component(ABC):
    def __init__(self):
        self.parent = None
        self.children = []
        self.uuid = uuid.uuid1()

    def _add_parent(self, parent):
        self.parent = parent

    def _add_child(self, child):
        self.children.append(child)

    def add(self, other):
        self._add_child(other)
        other.parent = self

    def event_callback(self, msg: str):
        try:
            self.parent.event_callback(msg)
        except AttributeError:
            pass

    def __str__(self):
        return type(self).__name__
