from core.components import Component


def formatlog(severity, who, loginfo, skill=None, params=None):
    return f"[{severity}], {who}, {loginfo}, {skill}, {params}"


class Logger(Component):
    def __init__(self, *args, **kwargs):
        super(Logger, self).__init__(*args, **kwargs)

    def build(self):
        pass

    def run(self):
        pass