'''
Class representing InnerModelJoint
Author: ksakash@github.com (Akash Kumar Singh)
'''

from innermodeltransform import InnerModelTransform
from innermodelvector import InnerModelVector

class InnerModelJoint (InnerModelTransform):
    def __init__ (self, id: str, lx: float, ly: float, lz: float, hx: float, hy: float, hz: float,
                  tx: float, ty: float, tz: float, rx: float, ry: float, rz: float, min: float =-float("inf"),
                  max: float = float("inf"), port: int = 0, axis: str = "z", home: float = 0,
                  parent: 'InnerModelTransform' = None):
        '''Constructor
        :param id: identifier of the joint
        :param lx, ly, lz: lower limit of the joint
        :param hx, hy, hz: higher limit of the joint
        :param tx, ty, tz: translation of the joint
        :param rx, ry, rz: rotation of the joint
        :param min: minimum value of the joint rotation
        :param max: maximum value of the joint rotation
        :param port: port of the joint
        :param axis: axis around which it revolves
        :param parent: parent of the given node
        '''

        super (InnerModelJoint, self).__init__ (id, 'static', tx, ty, tz, rx, ry, rz, 0, parent)
        self.min = min
        self.max = max
        self.home = home
        self.port = port
        self.axis = axis
        if axis is 'x':
            self.backl = lx
            self.backh = hx
            self.update (min, 0, 0, max, 0, 0)
        elif axis is 'y':
            self.backl = ly
            self.backh = hy
            self.update (0, min, 0, 0, max, 0)
        elif axis is 'z':
            self.backl = lz
            self.backh = hz
            self.update (0, 0, min, 0, 0, max)
        else:
            raise Exception ("internal error, no such axis %s", axis)
        self.backl = None
        self.backh = None

    def printT (self, verbose: bool):
        '''Print info about the given node'''

        print ("Joint: %s", self.id)
        if verbose:
            print (self.rtmat)

    def save (self, out, tabs: int):
        s = ""
        for _ in range (tabs):
            s += "\t"

        s += "<joint id=\"" + self.id + "\" port=\"" + self.port + "\" axis=\"" + self.axis + \
             "\" home=\"" + "%.9f" % self.home + "\" min=\"" + "%.9f" % self.min + "\" max=\"" + \
             "%.9f" % self.max + "\" tx=\"" + "%.9f" % self.tx + "\" ty=\"" + "%.9f" % self.tx + \
             "\" tz=\"" + "%.9f" % self.tz + "\" rx=\"" + "%.9f" % self.rx + "\" ry=\"" + \
             "%.9f" % self.ry + "\" rz=\"" + "%.9f" % self.rz + "\">\n"

        out.write (s)

        for child in self.children:
            child.save (out, tabs+1)

        s = ""

        for _ in range (tabs):
            s += "\t"

        s += "</joint>\n"
        out.write (s)

    def update (self, lx: float, ly: float, lz: float, hx: float, hy: float, hz: float):
        '''Update parameters about the given parameters'''

        if self.axis is 'x':
            self.backl = lx
            self.backh = hx
        elif self.axis is 'y':
            self.backl = ly
            self.backh = hy
        elif self.axis is 'z':
            self.backl = lz
            self.backh = hz
        self.fixed = True

    def getAngle (self) -> float:
        '''Returns the angle of rotation about the given axis'''

        if (self.axis is 'x'):
            return self.rx
        elif (self.axis is 'y'):
            return self.ry
        elif (self.axis is 'z'):
            return self.rz

    def setAngle (self, angle: float, force: bool = False) -> float:
        '''Set angle of rotation'''

        ret = angle

        if (angle > self.max):
            ret = self.max
        elif (angle < self.min):
            ret = self.min

        if (self.axis == "x"):
            self.rx = ret
            self.rtmat.set (ret, 0, 0, 0, 0, 0)
        elif (self.axis == "y"):
            self.ry = ret
            self.rtmat.set(0, ret, 0, 0, 0, 0)
        elif (self.axis == "z"):
            self.rz = ret
            self.rtmat.set(0, 0, ret, 0, 0, 0)
        else:
            raise Exception ("internal error, no such axis %s", self.axis)

        if (self.innerModel is not None):
            self.innerModel.cleanupTables()

        return ret

    def uninaryAxis (self) -> 'InnerModelVector':
        if self.axis is 'x':
            return InnerModelVector.vec3d (1, 0, 0)
        elif self.axis is 'y':
            return InnerModelVector.vec3d (0, 1, 0)
        elif self.axis is 'z':
            return InnerModelVector.vec3d (0, 0, 1)
        return InnerModelVector.vec3d (0, 0, 0)
