from innermodelnode import InnerModelNode
from innermodelvector import InnerModelVector

class InnerModelPlane (InnerModelNode):
    def __init__ (self, id: str, texture: str, width: float, height: float, depth: float, repeat: int,
                  nx: float, ny: float, nz: float, px: float, py: float, pz: float, collidable: bool,
                  parent: 'InnerModelNode' = None):
        super (InnerModelPlane, self).__init__(id, parent)
        self.normal = InnerModelVector.vec3d (nx, ny, nz)
        self.point = InnerModelVector.vec3d (px, py, pz)
        self.texture = texture
        self.width = width
        self.height = height
        self.depth = depth
        self.repeat = repeat

    def printT (self, verbose: bool):
        if verbose:
            print ("Plane:", self.id)
            print (self.normal)

    def udpate (self, nx=None, ny=None, nz=None, px=None, py=None, pz=None):
        if (nx is None):
            self.updateChildren()
        else:
            self.normal[0] = nx; self.normal[1] = ny; self.normal[2] = nz
            self.parent[0] = px; self.parent[1] = py; self.parent[2] = pz
            self.fixed = True

    # TODO
    def save (self, out, tabs: int):
        pass

    def copyNode (self, hash: dict, parent: 'InnerModelNode') -> 'InnerModelNode':
        ret = InnerModelPlane (self.id, self.texture, self.width, self.height, self.depth,
                               self.repeat, self.normal[0], self.normal[1], self.normal[2],
                               self.point[0], self.point[1], self.point[2], False, parent)
        ret.level = self.level
        ret.fixed = self.fixed
        ret.children = []
        ret.attributes = dict()
        hash[self.id] = ret
        ret.innerModel = parent.innerModel

        for child in self.children:
            ret.addChild(child.copyNode(hash, ret))
        return ret

