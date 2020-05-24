from innermodelnode import InnerModelNode
from innermodelrtmatrix import InnerModelRTMat

class InnerModelTransform(InnerModelNode):
    def __init__ (self, id: str, engine: str, tx: float, ty: float, tz: float, rx: float, ry: float,
                  rz: float, mass: float = 0, parent: 'InnerModelNode' = None):
        super (InnerModelTransform, self).__init__(id, parent)
        self.rtmat = InnerModelRTMat.getInnerModelRTMat (tx=tx, ty=ty, tz=tz, rx=rx, ry=ry, rz=rz)
        self.mass = mass

        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.rx = rx
        self.ry = ry
        self.rz = rz

        self.gui_translation = True
        self.gui_rotation = True

        self.engine = engine

    # print the rt matrix
    def printT (self, verbose):
        if verbose:
            print ("{} ({}, {})".format(self.id, self.rtmat.shape[0], self.rtmat.shape[1]))
            print (self.rtmat)

    # TODO: to save the model in a file
    def save (self, out, tabs):
        pass

    def updatePointers (self, tx, ty, tz, rx, ry, rz):
        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.fixed = False

    def updateTranslationPointers (self, tx, ty, tz):
        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.fixed = False

    def updateRotationPointers (self, rx, ry, rz):
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.fixed = False

    def update (self, tx=None, ty=None, tz=None, rx=None, ry=None, rz=None):
        if tx is None and self.fixed:
            self.updateChildren(); return
        self.tx = tx; self.ty = ty; self.tz = tz
        self.rx = rx; self.ry = ry; self.rz = rz

        self.rtmat.set (ox=self.rx, oy=self.ry, oz=self.rz,
                        x=self.tx, y=self.ty, z=self.tz)

    def copyNode (self, hash, parent) -> 'InnerModelNode':
        ret = InnerModelTransform (self.id, self.engine, self.tx, self.ty, self.tz,
                                self.rx, self.ry, self.rz, self.mass, self.parent)
        ret.level = self.level
        ret.fixed = self.fixed
        ret.children = []
        ret.attributes = dict()
        hash[self.id] = ret
        ret.innerModel = parent.innerModel

        for child in self.children:
            ret.addChild(child.copyNode(hash, ret))
        return ret
