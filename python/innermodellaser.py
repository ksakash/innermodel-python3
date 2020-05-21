import math
from innermodelnode import InnerModelNode
from innermodel import InnerModel
from innermodelvector import InnerModelVector

class InnerModelLaser (InnerModelNode):
    def __init__ (self, id: str, port: int, min: int, max: int, angle: float, measures: int,
                  ifconfig: str, innermodel: 'InnerModel', parent: 'InnerModelNode' = None):
        super (InnerModelLaser, self).__init__ (id, parent)
        self.port = port
        self.min = min
        self.max = max
        self.angle = angle
        self.measures = measures
        self.ifconfig = ifconfig
        self.innerModel = innermodel

    # TODO
    def save (self, out, tabs: int):
        pass

    def printT (self, verbose: bool):
        if verbose:
            print ("Laser")

    def udpate (self):
        self.updateChildren()

    def copyNode (self, hash: dict, parent: 'InnerModelNode') -> 'InnerModelNode':
        ret = InnerModelLaser (self.id, self.port, self.min, self.max, self.angle, self.measures,
                               self.ifconfig, self.innerModel, parent)
        ret.level = self.level
        ret.fixed = self.fixed
        ret.children = []
        ret.attributes = dict()
        hash[self.id] = ret
        ret.innerModel = parent.innerModel

        for child in self.children:
            ret.addChild(child.copyNode(hash, ret))
        return ret

    def laserTo (self, dest: str, r: float, alpha: float) -> 'InnerModelVector':
        p = InnerModelVector((3,))
        p[0] = r*math.sin (alpha)
        p[1] = 0
        p[2] = r*math.cos (alpha)
        return self.innerModel.transform (dest, self.id, p)
