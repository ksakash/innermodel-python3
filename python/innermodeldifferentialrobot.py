from innermodeltransform import InnerModelTransform

class InnerModelDifferentialRobot (InnerModelTransform):
    def __init__(self, id: str, tx: float, ty: float, tz: float, rx: float, ry: float, rz: float,
                 port: int = 0, noise: float = 0, collide: bool = False,
                 parent: 'InnerModelTransform' = None):
        super (InnerModelDifferentialRobot, self).__init__ (id, 'static', tx, ty, tz, rx, ry, rz, 0,
                                                            parent)
        self.port = port
        self.noise = noise
        self.collide = collide

    def copyNode (self, hash: dict, parent: 'InnerModelNode') -> 'InnerModelNode':
        ret = InnerModelDifferentialRobot (self.id, self.tx, self.ty, self.tz, self.rx, self.ry,
                                           self.rz, self.port, self.port, False, parent)
        ret.level = self.level
        ret.fixed = self.fixed
        ret.children = []
        ret.attributes = dict()
        hash[self.id] = ret
        ret.innerModel = parent.innerModel

        for child in self.children:
            ret.addChild(child.copyNode(hash, ret))
        return ret
