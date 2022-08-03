class Position:
    def __init__(self, x=0, y=0, z=0.1):
        self.x = x
        self.y = y
        self.z = z

    def to_list(self):
        return [self.x, self.y, self.z]


class Orientation:
    def __init__(self, x=0, y=0, z=0, w=0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def to_list(self):
        return [self.x, self.y, self.z, self.w]


class Pose:
    def __init__(self):
        self.position = Position()
        self.orientation = Orientation()

    def __str__(self):
        return ' '.join([str(x) for x in self.position.to_list()]) + ' ' + ' '.join([str(x) for x in self.orientation.to_list()])