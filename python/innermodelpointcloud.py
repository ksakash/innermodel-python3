from innermodelnode import InnerModelNode

class InnerModelPointCloud (InnerModelNode):
    def __init__ (self, id: str, parent: 'InnerModelNode' = None):
        super (InnerModelPointCloud, self).__init__ (id, parent)

    def __repr__ (self):
        s = "InnerModelPointCloud, id: {}".format (self.id)
        return s

    def save (self, out, tabs: int):
        s = ""
        for _ in range (tabs):
            s += "\t"
        s += "<pointcloud id=\"" + self.id + "\"/>\n"
        out.write (s)

    def printT (self, verbose: bool):
        if verbose:
            print ("Point Cloud: %s"%self.id)

    def update (self):
        self.updateChildren()
