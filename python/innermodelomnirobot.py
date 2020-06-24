from innermodeltransform import InnerModelTransform

class InnerModelOmniRobot (InnerModelTransform):
    def __init__ (self, id: str, tx: float, ty: float, tz: float,
                  rx: float, ry: float, rz: float, port: int = 0,
                  noise: float = 0, collide: bool = False,
                  parent: 'InnerModelTransform' = None):
        super (InnerModelOmniRobot, self).__init__ (id, 'static', tx, ty, tz, rx, ry, rz, 0, parent)
        self.port = port
        self.noise = noise
        self.collide = collide  # may be redundant
