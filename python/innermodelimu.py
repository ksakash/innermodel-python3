from innermodelnode import InnerModelNode

class InnerModelIMU (InnerModelNode):
    def __init__ (self, id: str, port: int, parent: 'InnerModelNode' = None):
        super (InnerModelIMU, self).__init__ (id=id, parent=parent)
        self.port = port

    # TODO
    def save (self, out, tabs: int):
        pass

    def printT (self, verbose: bool):
        if verbose:
            print ("IMU")

    def update (self):
        self.updateChildren()

    def copyNode (self, hash: dict, parent: 'InnerModelNode'):
        ret = InnerModelIMU (id=self.id, port=self.port, parent=self.parent)
        ret.level = self.level
        ret.fixed = self.fixed
        ret.attributes.clear()
        ret.children.clear()
        hash[id] = ret
        ret.innerModel = parent.innerModel

        for child in self.children:
            ret.addChild (child.copyNode(hash, ret))

        return ret
