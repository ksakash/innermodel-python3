'''
Class to represent a plane
Author: ksakash@github.com (Akash Kumar Singh)
'''

from innermodelnode import InnerModelNode
from innermodelvector import InnerModelVector

class InnerModelPlane (InnerModelNode):
    '''Class to represent a plane (but actually used for any box like object)'''

    def __init__ (self, id: str, texture: str, width: float, height: float, depth: float, repeat: int,
                  nx: float, ny: float, nz: float, px: float, py: float, pz: float, collidable: bool,
                  parent: 'InnerModelNode' = None):
        '''
        :param id: identifier of the plane
        :param texture: file path for the texture
        :param width: width of plane
        :param height: height of plane
        :param depth: depth of plane
        :param nx, ny, nz: normal of the plane
        :param px, py, pz: location of the center of plane
        :param collidable: if the plane has a collision mesh
        :param parent: parent node of the current plane
        '''

        super (InnerModelPlane, self).__init__(id, parent)
        self.normal = InnerModelVector.vec3d (nx, ny, nz)
        self.point = InnerModelVector.vec3d (px, py, pz)
        self.texture = texture
        self.width = width
        self.height = height
        self.depth = depth
        self.repeat = repeat # redundant
        self.collidable = collidable

    def __repr__ (self):
        ret = "normal: {}, point: {}, texture: {}, width: {}, height: {}, depth: {}".\
              format (self.normal, self.point, self.texture, self.width, self.height, self.depth)
        return ret

    def printT (self, verbose: bool):
        '''Print info about the current node'''

        if verbose:
            print ("Plane:", self.id)
            print (self.normal)

    def udpate (self, nx=None, ny=None, nz=None, px=None, py=None, pz=None):
        '''Update paramters of the current node'''

        if (nx is None):
            self.updateChildren()
        else:
            self.normal[0] = nx; self.normal[1] = ny; self.normal[2] = nz
            self.parent[0] = px; self.parent[1] = py; self.parent[2] = pz

    def save (self, out, tabs: int):
        s = ""
        for _ in range (tabs):
            s += "\t"

        s += "<plane id=\"" + self.id + "\" texture=\"" + self.texture + "\" size=\"" + \
             "%.3f" % self.width + "," + "%.3f" % self.height + "," + "%.3f" % self.depth + \
             "\" repeat=\"" + "%.3f" % self.repeat + "\" nx=\"" + "%.3f" % self.normal[0] + \
             "\" ny=\"" + "%.3f" % self.normal[1] + "\" nz=\"" + "%.3f" % self.normal[2] + \
             "\" px=\"" + "%.3f" % self.point[0] + "\" py=\"" + "%.3f" % self.point[1] + \
             "\" pz=\"" + "%.3f" % self.point[2] + "\" collide=\"" + "%.3f" % self.collidable + \
             "\" />\n"

        out.write (s)
