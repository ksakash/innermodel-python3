import numpy as np
from innermodelvector import InnerModelVector
from innermodelmatrix import InnerModelMatrix

class InnerModelRTMat(InnerModelMatrix):
    def init(self, ox: float, oy: float, oz: float, t: InnerModelVector):
        pass

    def set(self, ox: float, oy: float, oz: float, x: float, y: float, z: float):
        pass

    def setR(self, rot: InnerModelMatrix):
        pass

    def setR(self, ox: float, oy: float, oz: float):
        pass

    def setRX(self, ox: float):
        pass

    def setRY(self, oy: float):
        pass

    def setRZ(self, oz: float):
        pass

    def setTr(self, x: float, y: float, z: float):
        pass

    def setTr(self, t: InnerModelVector):
        pass

    def setRT(self, ox: float, oy: float, oz: float, t: InnerModelVector):
        pass

    def getTr(self) -> 'InnerModelVector':
        pass

    def getR() -> 'InnerModelMatrix':
        pass

    # TODO
    # Rot3DOX getRx() const

    def getRxValue() -> float:
        pass

    # TODO
    # Rot3DOY getRy() const;

    def getRyValue() -> float:
        pass

    # TODO
    # Rot3DOZ getRz() const;

    def getRzValue(self) -> float:
        pass

    def invert(self) -> 'InnerModelRTMat':
        pass

    def invertR() -> 'InnerModelMatrix':
        pass

    def serializeAsString() -> str:
        pass

class InnerModelRTCMat(InnerModelMatrix):
    pass
