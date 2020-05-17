from innermodelrtmatrix import InnerModelRTMat

class InnerModelNode (object):
    def __init__ (self, id, parent):
        self.rtmat = None
        self.id = id
        self.innerModel = None
        self.parent = None
        self.children = None
        self.attributes = None

    def printTree (self, s, verbose):
        pass

    def print (self, verbose):
        pass

    def update (self):
        pass

    def copyNode (self, hash, parent):
        pass

    def save (self, out, tabs):
        pass

    def setParent (self, parent):
        pass

    def addChild (self, child):
        pass

    def setFixed (self, f):
        pass

    def isFixed (self):
        pass

    def updateChildren (self):
        pass
