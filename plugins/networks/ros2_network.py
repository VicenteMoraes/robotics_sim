import docker
from plugins.plugin import Plugin


class ROS2Network(docker.client.NetworkCollection.model, Plugin):
    def __init__(self, *args, **kwargs):
        super(ROS2Network, self).__init__(*args, **kwargs)

    def run(self):
        pass
