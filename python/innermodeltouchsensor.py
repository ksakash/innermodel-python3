from innermodelnode import InnerModelNode

class InnerModelTouchSensor (InnerModelNode):
    def __init__ (self, id: str, stype: str, nx: float, ny: float, nz: float, min=-float("inf"),
                  max=float("inf"), port: int = 0, parent: 'InnerModelNode' =None):
        super (InnerModelTouchSensor, self).__init__ (id, parent)
        self.nx = nx; self.ny = ny; self.nz = nz
        self.min = min; self.max = max
        self.stype = stype
        self.port = port
        self.value = None

    def printT (self, verbose: bool):
        pass

    def save (self, out, tabs: int):
        pass

    def update (self):
        pass

    def getMeasure (self) -> float:
        return self.value
