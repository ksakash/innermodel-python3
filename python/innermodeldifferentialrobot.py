from innermodeltransform import InnerModelTransform

class InnerModelDifferentialRobot (InnerModelTransform):
    def __init__(self, id: str, tx: float, ty: float, tz: float,
                 rx: float, ry: float, rz: float, port: int = 0,
                 noise: float = 0, collide: bool = False,
                 parent: 'InnerModelTransform' = None):
        super (InnerModelDifferentialRobot, self).__init__ (id, 'static', tx, ty, tz, rx, ry, rz, 0,
                                                            parent)
        self.port = port
        self.noise = noise
        self.collide = collide # redundant

    def __repr__ (self):
        s = "InnerModelDifferentialRobot, id: {}, pos: [{}, {}, {}], orientation: [{}, {}, {}], \
             port: {}, noise: {}".format (self.id, self.tx, self.ty, self.tz, self.rx, self.ry, \
             self.rz, self.port, self.noise)
        return s
