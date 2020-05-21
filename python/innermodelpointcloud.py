from innermodelnode import InnerModelNode

class InnerModelPointCloud (InnerModelNode):
    def __init__ (self, id: str, parent: 'InnerModelNode' = None):
        super (InnerModelPointCloud, self).__init__ (id, parent)

    # TODO
    def save (self, out, tabs: int):
        pass

    def printT (self, verbose: bool):
        if verbose:
            print ("Point Cloud: %s", self.id)

    def update (self):
        self.updateChildren()

    def copyNode (self, hash: dict, parent: 'InnerModelNode') -> 'InnerModelNode':
        ret = InnerModelPointCloud (self.id, parent)
        ret.level = self.level
        ret.fixed = self.fixed
        ret.children = []
        ret.attributes = dict()
        hash[self.id] = ret
        ret.innerModel = parent.innerModel

        for child in self.children:
            ret.addChild(child.copyNode(hash, ret))
        return ret

