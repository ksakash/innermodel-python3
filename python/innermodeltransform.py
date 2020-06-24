'''
Class representing transformation (tx, ty, tz, rx, ry, rz)
Author: ksakash@github.com (Akash Kumar Singh)
'''

from innermodelnode import InnerModelNode
from innermodelrtmatrix import InnerModelRTMat

class InnerModelTransform(InnerModelNode):
    '''Class representing transformation (tx, ty, tz, rx, ry, rz)'''

    def __init__ (self, id: str, engine: str, tx: float, ty: float, tz: float, rx: float, ry: float,
                  rz: float, mass: float = 0, parent: 'InnerModelNode' = None):
        '''Constructor
        :param id: identifier
        :param engine:
        :param tx, ty, tz: translation
        :param rx, ry, rz: rotation
        :param mass: weight
        :param parent: parent of the node
        '''

        super (InnerModelTransform, self).__init__(id, parent)
        self.rtmat = InnerModelRTMat.getInnerModelRTMat (tx=tx, ty=ty, tz=tz, rx=rx, ry=ry, rz=rz)
        self.mass = mass

        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.rx = rx
        self.ry = ry
        self.rz = rz

        self.gui_translation = True # may be redundant
        self.gui_rotation = True # may be redundant

        self.engine = engine # may be redundant

    def __repr__ (self):
        ret = "id: {}, mass: {}, tx: {}, ty: {}, tz: {}, rx: {}, ry: {}, rz: {}"\
              .format (self.id, self.mass, self.tx, self.ty, self.tz, self.rx, self.ry, self.rz)
        return ret

    # print the rt matrix
    def printT (self, verbose): # redundant if __repr__ is used
        '''Print info about object'''

        if verbose:
            print ("{} {}".format(self.id, self.rtmat.shape))
            print (self.rtmat)

    # TODO: to save the model in a file
    def save (self, out, tabs):
        '''Save info to a doc'''

        pass

    def updateTranslation (self, tx, ty, tz): # may be redundant
        '''Update translation parameters'''

        self.tx = tx
        self.ty = ty
        self.tz = tz

    def updateRotation (self, rx, ry, rz): # may be redundant
        '''Update rotation parameters'''

        self.rx = rx
        self.ry = ry
        self.rz = rz

    def update (self, tx=None, ty=None, tz=None, rx=None, ry=None, rz=None):
        '''Update the paramters of the object'''

        if tx is not None:
            self.tx = tx
        if ty is not None:
            self.ty = ty
        if tz is not None:
            self.tz = tz
        if rx is not None:
            self.rx = rx
        if ry is not None:
            self.ry = ry
        if rz is not None:
            self.rz = rz

        self.rtmat.set (ox=self.rx, oy=self.ry, oz=self.rz,
                        x=self.tx, y=self.ty, z=self.tz)

        # self.updateChildren()

    def copyNode (self, hash, parent) -> 'InnerModelNode':
        '''Return a copy of the node'''

        ret = InnerModelTransform (self.id, self.engine, self.tx, self.ty, self.tz,
                                   self.rx, self.ry, self.rz, self.mass, self.parent)
        ret.level = self.level
        ret.children = []
        ret.attributes = dict()
        hash[self.id] = ret
        ret.innerModel = parent.innerModel

        for child in self.children:
            ret.addChild(child.copyNode(hash, ret))
        return ret
