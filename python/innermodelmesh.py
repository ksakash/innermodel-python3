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
        self.renderMode = render # may be redundant
        self.collidable = collidable # may be redundant

    def save (self, out, tabs):
        s = ""
        for _ in range (tabs):
            s += "\t"

        s += "<mesh id=\"" + self.id + "\"" + " file=\"" + self.meshPath + "\" scale=\"" + \
             "%.9f" % self.scalex + "," + "%.9f" % self.scalex + "," + "%.9f" % self.scalex + \
             "\" tx=\"" + "%.9f" % self.tx + "\" ty=\"" + "%.9f" % self.ty + "\" tz=\"" + \
             "%.9f" % self.tz + "\" rx=\"" + "%.9f" % self.rx + "\" ry=\"" + "%.9f" % self.ry + \
             "\" rz=\"" + "%.9f" % self.rz + "\" collide=\"" + "%.9f" % self.collidable << "\" />\n"
        out.write (s)

    def __repr__ (self):
        scale = "scale: [{}, {}, {}]".format (self.scalex, self.scaley, self.scalez)
        translation = "translation: [{}, {}, {}]".format (self.tx, self.ty, self.tz)
        rotation = "rotation: [{}, {}, {}]".format (self.rx, self.ry, self.rz)
        meshPath = "meshPath: {}".format (self.meshPath)

        ret = scale + ", " + translation + ", " + rotation + ", " + meshPath
        return ret

    def printT (self, verbose): # may be redundant
        if (verbose):
            print ("Mesh:", self.id)

    def update (self): # may be redundant
        self.updateChildren()

    def setScale (self, x, y, z):
        self.scalex = x
        self.scaley = y
        self.scalez = z

    def normalRendering (self) -> bool: # may be redundant
        return self.renderMode == 'NormalRendering'

    def wireFrameRendering (self) -> bool: # may be redudant
        return self.renderMode == 'WireFrameRendering'
