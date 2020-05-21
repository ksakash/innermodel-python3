from innermodeltransform import InnerModelTransform
from innermodelvector import InnerModelVector

class InnerModelJoint (InnerModelTransform):
    def __init__ (self, id: str, lx: float, ly: float, lz: float, hx: float, hy: float, hz: float,
                  tx: float, ty: float, tz: float, rx: float, ry: float, rz: float, min: float =-float("inf"),
                  max: float = float("inf"), port: int = 0, axis: str = "z", home: float = 0,
                  parent: 'InnerModelTransform' = None):
        super (InnerModelJoint, self).__init__ (id, 'static', tx, ty, tz, rx, ry, rz, 0, parent)
        self.min = min
        self.max = max
        self.home = home
        self.port = port
        self.axis = axis
        if axis is 'x':
            self.backl = lx
            self.backh = hx
            self.update (min, 0, 0, max, 0, 0)
        elif axis is 'y':
            self.backl = ly
            self.backh = hy
            self.update (0, min, 0, 0, max, 0)
        elif axis is 'z':
            self.backl = lz
            self.backh = hz
            self.update (0, 0, min, 0, 0, max)
        else:
            raise Exception ("internal error, no such axis %s", axis)
        self.backl = None
        self.backh = None

    def printT (self, verbose: bool):
        print ("Joint: %s", self.id)
        if verbose:
            print (self.rtmat)

    # TODO
    def save (self, out, tabs: int):
        pass

    def update (self, lx: float, ly: float, lz: float, hx: float, hy: float, hz: float):
        if self.axis is 'x':
            self.backl = lx
            self.backh = hx
        elif self.axis is 'y':
            self.backl = ly
            self.backh = hy
        elif self.axis is 'z':
            self.backl = lz
            self.backh = hz
        self.fixed = True

    def getAngle (self) -> float:
        if (self.axis is 'x'):
            return self.rx
        elif (self.axis is 'y'):
            return self.ry
        elif (self.axis is 'z'):
            return self.rz

    def setAngle (self, angle: float, force: bool = False) -> float:
        ret = angle

        if (angle > self.max):
            ret = self.max
        elif (angle < self.min):
            ret = self.min

        if (self.axis == "x"):
            self.rx = ret
            self.rtmat.set (ret, 0, 0, 0, 0, 0)
        elif (self.axis == "y"):
            self.ry = ret
            self.rtmat.set(0, ret, 0, 0, 0, 0)
        elif (self.axis == "z"):
            self.rz = ret
            self.rtmat.set(0, 0, ret, 0, 0, 0)
        else:
            raise Exception ("internal error, no such axis %s", self.axis)

        if (self.innerModel is not None):
            self.innerModel.cleanupTables()

        return ret

    def uninaryAxis (self) -> 'InnerModelVector':
        if self.axis is 'x':
            return InnerModelVector.vec3d (1, 0, 0)
        elif self.axis is 'y':
            return InnerModelVector.vec3d (0, 1, 0)
        elif self.axis is 'z':
            return InnerModelVector.vec3d (0, 0, 1)
        return InnerModelVector.vec3d (0, 0, 0)

    def copyNode (self, hash: dict, parent: 'InnerModelNode') -> 'InnerModelNode':
        if self.axis is 'x':
            ret = InnerModelJoint (id, self.backl, 0, 0, self.backh, 0, 0, self.tx, self.ty, self.tz,
                                   self.rx, 0, 0, self.min, self.max, self.port, self.axis, self.home,
                                   parent)
        elif self.axis is 'y':
            ret = InnerModelJoint (id, self.backl, 0, 0, self.backh, 0, 0, self.tx, self.ty, self.tz,
                                   0, self.ry, 0, self.min, self.max, self.port, self.axis, self.home,
                                   parent)
        elif self.axis is 'z':
            ret = InnerModelJoint (id, self.backl, 0, 0, self.backh, 0, 0, self.tx, self.ty, self.tz,
                                   0, 0, self.rz, self.min, self.max, self.port, self.axis, self.home,
                                   parent)
        else:
            raise Exception ("invalid axis: %s", self.axis)

        ret.level = self.level
        ret.fixed = self.fixed
        ret.children = []
        ret.attributes = dict()
        hash[self.id] = ret
        ret.innerModel = parent.innerModel

        for child in self.children:
            ret.addChild(child.copyNode(hash, ret))
        ret.setAngle (self.getAngle())
        return ret
