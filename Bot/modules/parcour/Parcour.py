from bin.Publisher import Publisher


class Parcour(Publisher):
    def __init__(self):
        Publisher.__init__(self)
        self.start()

    def start(self):
        self.send("motor", "Hallo vanuit Parcour!")


parcour = Parcour()
parcour.start()