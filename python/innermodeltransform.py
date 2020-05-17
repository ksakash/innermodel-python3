from innermodelnode import InnerModelNode

class InnerModelTransform(InnerModelNode):
    def __init__ (self, id, engine, tx, ty, tz, rx, ry, rz, mass, parent):
        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.rx = rx
        self.ry = ry
        self.rz = rz

        self.mass = mass

        self.backtX = None
        self.backtY = None
        self.backtZ = None
        self.backrX = None
        self.backrY = None
        self.backrZ = None

        self.gui_translation = None
        self.gui_rotation = None

        self.engine = engine

    def print (self, verbose):
        pass

    def save (self, out, tabs):
        pass

    def updatePointers (self, tx, ty, tz, rx, ry, rz):
        pass

    def updateTranslationPointers (self, tx, ty, tz):
        pass

    def updateRotationPointers (self, rx, ry, rz):
        pass

    def update (self, tx, ty, tz, rx, ry, rz):
        pass
