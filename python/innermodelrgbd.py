from innermodelcamera import InnerModelCamera
# from innermodel import InnerModel

class InnerModel (object):
    pass

class InnerModelRGBD (InnerModelCamera):
    def __init__(self, id: str, width: float, height: float, focal: float, noise: float, port: int,
                 ifconfig: str, innermodel: 'InnerModel', parent: 'InnerModelNode' = None):
        super (InnerModelRGBD, self).__init__ (id, width, height, focal, innermodel, parent)
        self.noise = noise
        self.port = port
        self.ifconfig = ifconfig

    def save (self, out, tabs: int):
        s = ""
        for _ in range (tabs):
            s += "\t"
        s += "<rgbd id=\"" + self.id + "\" width=\"" + "%.9f" % self.width + "\" height=\"" + \
             "%.9f" % self.height + "\" focal=\"" + "%.9f" % self.focal + "\" port=\"" + self.port \
             + "\" ifconfig=\"" + self.ifconfig + "\" noise=\"" + "%.9f" % self.noise + "\" />\n"
        out.write (s)
