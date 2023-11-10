import sys
from abc import abstractmethod
from core.components import Component, ProjectPath
from core.envs.baseenv import Environment
from docker import DockerClient
from plugins.loggers.docker_logger import DockerLogger
import docker
import os
import subprocess
from paramiko.ssh_exception import ChannelException
from threading import Thread


class Module(Component):
    def __init__(self, priority: int = 5):
        # Priorities: 0 - highest, 5 - lowest
        super(Module, self).__init__()
        self.priority = priority

    def run(self):
        for child in self.children:
            try:
                child.run()
            except AttributeError:
                pass

    def build(self):
        for child in self.children:
            try:
                child.build()
            except AttributeError:
                pass

    def stop(self):
        for child in self.children:
            try:
                child.stop()
            except AttributeError:
                pass

    def __repr__(self):
        try:
            return f"{str(self)} plugin attached to Trial {str(self.parent)}, ID: {self.parent.trial_id}"
        except AttributeError:
            return f"{str(self)} plugin attached to Parent {str(self.parent)}, ID: {self.parent.uuid}"

    def __gt__(self, other):
        return self.priority > other.priority

    def __lt__(self, other):
        return self.priority < other.priority

    def __eq__(self, other):
        return self.priority == other.priority


class Plugin(Module):
    ssh_hash_dict = {}

    def __init__(self, docker_client: DockerClient, path: str, command: str, tag: str, ssh_host: str = None, ssh_pass: str = '',
                 dockerfile: str = "Dockerfile", auto_remove: bool = True, container_name: str = "", network=None,
                 run_children: bool = True, *args, **kwargs):
        super(Plugin, self).__init__(*args, **kwargs)
        self.docker_client = docker_client
        self.path = path  # Path to dockerfile directory
        self.tag = tag
        self.dockerfile = dockerfile  # Name of dockerfile file
        self.auto_remove = auto_remove
        self.image = None
        self.logs_thread = None
        self.container = None
        self.command = command
        self.network = network
        self.ssh_host = ssh_host
        self.ssh_pass = ssh_pass
        self.ssh_params = None
        self.run_children = run_children
        self._start_logger = None
        self.mounts = []
        self.env = Environment()
        self.env["TAG"] = self.tag
        self.container_name = container_name if container_name else f"{self.tag}_container"

    def add_mount(self, source: str, target: str, mount_type: str = 'bind', **mount_kwargs):
        if self.ssh_host is not None and mount_type == 'bind':
            if str(ProjectPath) in source:
                source = source.replace(str(ProjectPath), f'/home/{self.ssh_host.split("@")[0]}/.robotics_sim_files/')

        mnt = docker.types.Mount(target=target, source=source, type=mount_type, **mount_kwargs)
        self.mounts.append(mnt)
        return mnt

    def load_gui(self):
        self.load_gui_env()
        self.add_mount(source="/tmp/.X11-unix/", target="/tmp/.X11-unix/")

    def load_gui_env(self):
        self.env["DISPLAY"] = os.environ['DISPLAY']
        self.env["QT_GRAPHICSSYSTEM"] = "native"
        self.env["QT_X11_NO_MITSHM"] = "1"
        self.env["XAUTH_LIST"] = subprocess.run(["xauth", "list"], text=True, capture_output=True).stdout

    def connect(self, network):
        network.connect(self.container)

    def build(self, **build_kwargs):
        self.image, build_logs = self.docker_client.images.build(path=self.path, dockerfile=self.dockerfile,
                                                                      tag=self.tag, **build_kwargs)
        self.logs_thread = Thread(target=self.print_logs, args=[build_logs])
        self.logs_thread.run()
        super(Plugin, self).build()

    def run(self, **run_kwargs):
        if self.network:
            run_kwargs['network'] = self.network.name
        self.container = self.docker_client.containers.run(image=self.tag, command=self.command, detach=True, tty=True,
                                                           name=self.container_name, mounts=self.mounts,
                                                           environment=self.env.to_list(), auto_remove=self.auto_remove,
                                                           **run_kwargs)
        try:
            self._start_logger()
        except TypeError:
            pass
        super(Plugin, self).run()

    def add_logger(self, target: str = "", write_to_file: bool = False, filename: str = '', update_interval: float = 15,
                   log_kwargs: dict = None, timeout: float = 15*60, add_logger_func: bool = True,
                   *logger_args, **logger_kwargs):
        logger = DockerLogger(target=target, write_to_file=write_to_file, filename=filename,
                              update_interval=update_interval, log_kwargs=log_kwargs, timeout=timeout,
                              *logger_args, **logger_kwargs)
        self.add(logger)
        if add_logger_func:
            self._start_logger = logger.start_timer

    def kill(self, signal=None):
        self.container.kill(signal)

    def stop(self):
        while True:
            try:
                self.container.stop()
                break
            except docker.errors.NotFound:
                break
            except ChannelException:
                print('Paramiko broken channel')
                continue
        super(Plugin, self).stop()

    def print_logs(self, log_generator):
        print('\x1b[6;30;42m' + f"Building image for tag: {self.tag}" + '\x1b[0m')
        for log in log_generator:
            try:
                print(log['stream'])
            except KeyError:
                print(log)
