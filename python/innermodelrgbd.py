from innermodelcamera import InnerModelCamera
# from innermodel import InnerModel

class InnerModel (object):
    pass

class InnerModelRGBD (InnerModelCamera):
    def __init__(self, id: str, width: float, height: float, focal: float, noise: float, port: int,
                 ifconfig: str, innermodel: 'InnerModel', parent: 'InnerModelNode' = None):
        super (InnerModelRGBD, self).__init__ (id, width, height, focal, innermodel, parent)
        self.noise = noise
        self.port = port
        self.ifconfig = ifconfig

    # TODO
    def save (self, out, tabs: int):
        pass

    def copyNode (self, hash: dict, parent: 'InnerModelNode') -> 'InnerModelNode':
        ret = InnerModelRGBD (self.id, self.width, self.height, self.focal, self.noise, self.port,
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

