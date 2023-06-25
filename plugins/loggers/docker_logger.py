from plugins.loggers.logger import Logger, formatlog
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
        self.time = None

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
            self.stop_timer(target_reached=True)

    def write_logs(self, msg: str = ''):
        if not self.filename:
            print("No filename provided. Aborting writing logs")
            return
        with open(ProjectPath/'logs'/self.filename, 'w') as wf:
            try:
                wf.writelines(self.logs)
                if msg:
                    wf.writelines(f"\n{msg}\n")
            except TypeError:
                pass

    def stop_timer(self, target_reached: bool = False):
        self.timer.stop()
        if self.timeout_stop:
            if not target_reached:
                self.write_logs(msg=formatlog("WARN", 'logger', "TIMEOUT"))
            self.event_callback(msg="SHUTDOWN")

    def start_timer(self):
        self.time = time()
        self.timer.start()
