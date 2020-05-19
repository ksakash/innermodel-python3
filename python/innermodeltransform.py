from innermodelnode import InnerModelNode
from innermodelrtmatrix import InnerModelRTMat

class InnerModelTransform(InnerModelNode):
    def __init__ (self, id, engine, tx, ty, tz, rx, ry, rz, mass, parent):
        super (InnerModelTransform, self).__init__(id, parent)
        self.rtmat = InnerModelRTMat.getInnerModelRTMat (tx=tx, ty=ty, tz=tz, rx=rx, ry=ry, rz=rz)
        self.mass = mass

        self.backtX = tx
        self.backtY = ty
        self.backtZ = tz
        self.backrX = rx
        self.backrY = ry
        self.backrZ = rz

        self.tx = self.ty = self.tz = self.rx = self.ry = self.rz = None

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

    def updateRotationPointers (self, rx, ry, rz):
        self.rx = rx
        self.ry = ry
        self.rz = rz

    def update (self, tx=None, ty=None, tz=None, rx=None, ry=None, rz=None):
        if tx is None:
            if (not self.fixed):
                if (self.tx) is not None:
                    self.backtX = self.tx
                if (self.ty) is not None:
                    self.backtY = self.ty
                if (self.tz) is not None:
                    self.backtZ = self.tz
                if (self.rx) is not None:
                    self.backrX = self.rx
                if (self.ry) is not None:
                    self.backrY = self.ry
                if (self.rz) is not None:
                    self.backrZ = self.rz
                self.rtmat.set (ox=self.backrX, oy=self.backrY, oz=self.backrZ,
                                x=self.backtX, y=self.backtY, z=self.backtZ)
            self.updateChildren()
        else:
            self.backtX = tx
            self.backtY = ty
            self.backtZ = tz
            self.backrX = rx
            self.backrY = ry
            self.backrZ = rz
            self.rtmat.set (ox=self.backrX, oy=self.backrY, oz=self.backrZ,
                            x=self.backtX, y=self.backtY, z=self.backtZ)
            self.fixed = True

    def copyNode (self, hash, parent) -> 'InnerModelNode':
        ret = InnerModelTransform (self.id, self.engine, self.backtX, self.backtY, self.backtZ,
                                 self.backrX, self.backrY, self.backrZ, self.mass, self.parent)
        ret.level = self.level
        ret.fixed = self.fixed
        ret.children = []
        ret.attributes = dict()
        hash[self.id] = ret
        ret.innerModel = parent.innerModel

        for child in self.children:
            ret.addChild(child.copyNode(hash, ret))
        return ret
