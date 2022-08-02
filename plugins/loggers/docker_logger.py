from plugins.loggers.logger import Logger
from core.components import ProjectPath
from core.timer import RepeatedTimer


class DockerLogger(Logger):
    def __init__(self, target: str, write_to_file: bool = False, filename: str = '',
                 update_interval: float = 1, log_args: list = None, *args, **kwargs):
        super(DockerLogger, self).__init__(*args, **kwargs)
        self.target = target
        self.logs = None
        self.success = False
        self.write_to_file = write_to_file
        self.filename = filename
        log_args = log_args if log_args is not None else []
        self.timer = RepeatedTimer(update_interval, self.update, *log_args)

    def update(self, **log_kwargs):
        if not self.parent:
            return

        try:
            self.logs = self.parent.container.logs(**log_kwargs).decode()
        except AttributeError:
            return

        self.success = self.success or self.target in self.logs
        if self.write_to_file:
            self.write_logs()

    def write_logs(self):
        with open(ProjectPath/'logs'/self.filename, 'w') as wf:
            wf.writelines(self.logs)
