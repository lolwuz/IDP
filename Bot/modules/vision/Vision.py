from bin.Publisher import Publisher


class Vision(Publisher):

    def __init__(self):
        Publisher.__init__(self)

    def start(self):
        print("start called")
        self.send("vision_")

