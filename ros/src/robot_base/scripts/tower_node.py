from base.base_node import BaseNode


class TowerNode(BaseNode):

    def __init__(self):
        """ Node for finding and driving to the tower """
        super(TowerNode, self).__init__("tower", False, 24)

    def update(self):
        """ Go to the tower """
        super(TowerNode, self).update()
    #
    #     self.clamp()
    #
    # def clamp(self):
