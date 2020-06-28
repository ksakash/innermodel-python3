from innermodelnode import InnerModelNode

class InnerModelIMU (InnerModelNode):
    def __init__ (self, id: str, port: int, parent: 'InnerModelNode' = None):
        super (InnerModelIMU, self).__init__ (id=id, parent=parent)
        self.port = port

    def save (self, out, tabs: int):
        s = ""
        for _ in range (tabs):
            s += "\t"
        s += "<imu id=\"" + self.id + "\" />\n"
        out.write (s)

    def __repr__ (self) -> str:
        ret = "id: {}, port: {}".format (self.id, self.port)
        return ret

    def printT (self, verbose: bool):
        if verbose:
            print ("IMU")

    def update (self):
        self.updateChildren()
