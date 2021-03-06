from collections import UserDict


class Environment(UserDict):
    def to_list(self):
        if len(self) == 0:
            return [""]
        lst = []
        for key, value in self.items():
            lst.append(f"{key}={value}")
        return lst
