from innermodelnode import InnerModelNode

class InnerModelMesh (InnerModelNode):
    def __init__ (self, id, meshPath, scalex, scaley, scalez,
                render, tx, ty, tz, rx, ry, rz, collidable, parent):
        super (InnerModelMesh, self).__init__ (id, parent)
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
        self.renderMode = render
        self.collidable = collidable

    # TODO
    def save (self, out, tabs):
        pass

    def printT (self, verbose):
        if (verbose):
            print ("Mesh:", self.id)

    def update (self):
        self.updateChildren()

    def setScale (self, x, y, z):
        self.scalex = x
        self.scaley = y
        self.scalez = z

    def normalRendering (self) -> bool:
        return self.renderMode == 'NormalRendering'

    def wireFrameRendering (self):
        return self.renderMode == 'WireFrameRendering'

    def copyNode (self, hash, parent) -> 'InnerModelMesh':
        ret = InnerModelMesh (self.id, self.meshPath, self.scalex, self.scaley, self.scalez,
                              self.renderMode, self.tx, self.ty, self.tz, self.rx, self.ry,
                              self.rz, self.collidable, self.parent)
        ret.level = self.level
        ret.fixed = self.fixed
        ret.children = []
        ret.attributes = dict()
        hash[id] = ret
        ret.innerModel = parent.innerModel

        for child in self.children:
            ret.addChild (child.copyNode(hash, ret))

        return ret
