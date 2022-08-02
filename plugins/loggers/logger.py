from plugins.plugin import Plugin


class Logger(Plugin):
    def __init__(self, *args, **kwargs):
        super(Logger, self).__init__(*args, **kwargs)

    def build(self):
        pass

    def run(self):
        pass