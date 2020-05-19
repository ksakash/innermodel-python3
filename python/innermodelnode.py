class InnerModelNode (object):
    def __init__ (self, id: str, parent: 'InnerModelNode'):
        self.rtmat = None
        self.id = id
        self.innerModel = None
        self.parent = parent
        if self.parent is not None:
            self.level = parent.level + 1
        else:
            self.level = 0
        self.children = []
        self.attributes = None
        self.fixed = None

    # can also use __repr__ instead
    def printTree (self, s: str, verbose: bool):
        print (s, self.id, self.level, len(self.children))

        for child in self.children:
            if verbose:
                child.printT (verbose)
            child.printTree (s, verbose)

    # abstract
    def printT (self, verbose: bool):
        pass

    # abstract
    def update (self):
        pass

    # abstract
    def copyNode (self, hash, parent):
        pass

    # abstract
    def save (self, out, tabs):
        pass

    def setParent (self, parent: 'InnerModelNode'):
        self.parent = parent
        self.level = parent.level + 1

    def addChild (self, child: 'InnerModelNode'):
        child.innerModel = self.innerModel
        if child in self.children:
            self.children.append(child)
        child.parent = self

    def setFixed (self, f: bool):
        self.fixed = f

    def isFixed (self) -> bool:
        return self.fixed

    def updateChildren (self):
        for child in self.children:
            child.update()
