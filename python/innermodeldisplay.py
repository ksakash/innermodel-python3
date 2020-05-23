from innermodelnode import InnerModelNode
from innermodelvector import InnerModelVector

class InnerModelDisplay (InnerModelNode):
    def __init__ (self, id: str, port: int, texture: str, width: float, height: float, depth: float,
                  repeat: int, nx: float, ny: float, nz: float, px: float, py: float, pz: float,
                  collidable: bool, parent: 'InnerModelNode' = None):
        super (InnerModelDisplay, self).__init__ (id, parent)
        self.normal = InnerModelVector.vec3d (nx, ny, nz)
        self.point = InnerModelVector.vec3d (px, py, pz)
        self.port = port
        self.repeat = repeat
        self.texture = texture
        self.width = width
        self.height = height
        self.depth = depth
        self.collidable = collidable

    def updateTexture (self, texture: str):
        self.texture = texture

    def printT (self, verbose: bool):
        if verbose:
            print ("Display: ", self.id)
            print (self.normal)

    # TODO
    def save (self, out, tabs: int):
        pass

    def update (self, nx: float = None, ny: float = None, nz: float = None, px: float = None,
                py: float = None, pz: float = None):
        if nx is None:
            self.updateChildren()
        else:
            self.normal[0] = nx; self.normal[1] = ny; self.normal[2] = nz
            self.point[0] = nx; self.point[1] = ny; self.point[2] = nz

    def copyNode (self, hash: dict, parent: 'InnerModelNode') -> 'InnerModelNode':
        ret = InnerModelDisplay (self.id, self.port, self.width, self.height, self.depth, self.repeat,
                                 self.normal[0], self.normal[1], self.normal[2], self.point[0],
                                 self.point[1], self.point[2], self.collidable, parent)
        ret.level = self.level
        ret.fixed = self.fixed
        ret.children = []
        ret.attributes = dict()
        hash[self.id] = ret
        ret.innerModel = parent.innerModel

        for child in self.children:
            ret.addChild(child.copyNode(hash, ret))
        return ret
