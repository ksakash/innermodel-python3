from innermodelnode import InnerModelNode

class InnerModelMesh (InnerModelNode):
    def __init__ (self, id, meshPath, scalex, scaley, scalez,
                render, tx, ty, tz, rx, ry, rz, collidable, parent):
        self.meshPath = meshPath
        self.scalex = scalex
        self.scaley = scaley
        self.scalez = scalez
        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.rx = rx
        self.ry = ry
        self.rz = rz

    def save (self, out, tabs):
        pass

    def print (self, verbose):
        pass

    def update (self):
        pass

    def setScale (self, x, y, z):
        pass

    def normalRendering (self):
        pass

    def wireFrameRendering (self):
        pass

    def copyNode (self, hash, parent):
        pass

