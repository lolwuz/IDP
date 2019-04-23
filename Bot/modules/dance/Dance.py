from bin.Publisher import Publisher


class Dance(Publisher):
    def __init__(self):
        Publisher.__init__(self)

    def start(self):
        self.send("motor", "Hallo vanuit Dance!")