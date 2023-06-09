from plugins.loggers.logger import Logger
from core.components import ProjectPath
from core.timer import RepeatedTimer
from time import time
import requests


class DockerLogger(Logger):
    def __init__(self, target: str, write_to_file: bool = False, filename: str = '', timeout_stop: bool = False,
                 update_interval: float = 1, log_args: list = None, timeout: float = None, *args, **kwargs):
        super(DockerLogger, self).__init__(*args, **kwargs)
        self.target = target
        self.logs = None
        self.write_to_file = write_to_file
        self.filename = filename
        self.timeout = timeout
        self.timeout_stop = timeout_stop
        log_args = log_args if log_args is not None else []
        self.timer = RepeatedTimer(update_interval, self.update, *log_args)
        self.time = time()

    def update(self, **log_kwargs):
        if self.timeout is not None:
            if time() - self.time >= self.timeout:
                self.stop_timer()
                return
        if not self.parent:
            return

        try:
            self.logs = self.parent.container.logs(**log_kwargs).decode()
        except AttributeError:
            return
        except requests.exceptions.HTTPError:
            self.stop_timer()

        if self.write_to_file:
            self.write_logs()

        if self.target and self.target in self.logs:
            self.stop_timer()

    def write_logs(self):
        if not self.filename:
            print("No filename provided. Aborting writing logs")
            return
        with open(ProjectPath/'logs'/self.filename, 'w') as wf:
            wf.writelines(self.logs)

    def stop_timer(self):
        self.timer.stop()
        if self.timeout_stop:
            self.parent_stop()
