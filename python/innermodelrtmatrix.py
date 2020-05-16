import numpy as np
from innermodelvector import InnerModelVector
from innermodelmatrix import InnerModelMatrix
from innermodelrotmatrix import Rot3DOX, Rot3DOY, Rot3DOZ
from innermodelrotmatrix import Rot3DCOX, Rot3DCOY, Rot3DCOZ

class InnerModelRTMat(InnerModelMatrix):

    @staticmethod
    def getInnerModelRTMat(tx: float, ty: float, tz: float, rx: float, ry: float, rz: float) -> 'InnerModelRTMat':
        mat = InnerModelRTMat((4,4))
        mat.Rx = Rot3DOX.getRot3DOX(alpha=rx)
        mat.Ry = Rot3DOY.getRot3DOY(alpha=ry)
        mat.Rz = Rot3DOZ.getRot3DOZ(alpha=rz)
        R = mat.Rx.dot(mat.Ry.dot(mat.Rz))
        np.copyto(mat.R, R)
        mat.Tr = InnerModelVector.vec3d (tx, ty, tz)
        return mat

    def __new__(cls, *args, **kwargs):
        return super(InnerModelRTMat, cls).__new__(cls, *args, **kwargs)

    def __init__(self, *args, **kwargs):
        self.Rx = Rot3DOX.getRot3DOX(alpha=0)
        self.Ry = Rot3DOY.getRot3DOY(alpha=0)
        self.Rz = Rot3DOZ.getRot3DOZ(alpha=0)
        self.R = InnerModelMatrix.identity(3)
        self.Tr = InnerModelVector.vec3d(0, 0, 0)
        self.do_inject()

    def init(self, ox: float, oy: float, oz: float, t: InnerModelVector):
        np.copyto(self.Tr, t)
        self.Rx.update(ox)
        self.Ry.update(oy)
        self.Rz.update(oz)
        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto(self.R, R)
        self.do_inject()

    def do_inject (self):
        np.copyto (self[:3, :3], self.R)
        np.copyto (self[:3, 3], self.Tr)
        self[3][0] = 0.
        self[3][1] = 0.
        self[3][2] = 0.
        self[3][3] = 1.

    def set(self, ox: float, oy: float, oz: float, x: float, y: float, z: float):
        self.Tr[0] = x
        self.Tr[1] = y
        self.Tr[2] = z

        self.Rx.update(ox)
        self.Ry.update(oy)
        self.Rz.update(oz)

        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto(self.R, R)
        self.do_inject()

    def setR(self, ox: float, oy: float, oz: float, rot: InnerModelMatrix = None):
        if rot is not None:
            assert (self.R.shape == rot.shape)
            np.copyto(self.R, rot)
        else:
            self.Rx.update(ox)
            self.Ry.update(oy)
            self.Rz.update(oz)
            R = self.Rx.dot(self.Ry.dot(self.Rz))
            np.copyto(self.R, R)
        self.do_inject()

    def setRX(self, ox: float):
        self.Rx.update(ox)
        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto(self.R, R)
        self.do_inject()

    def setRY(self, oy: float):
        self.Ry.update(oy)
        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto(self.R, R)
        self.do_inject()

    def setRZ(self, oz: float):
        self.Rz.update(oz)
        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto(self.R, R)
        self.do_inject()

    def setTr(self, t: InnerModelVector = None):
        if t is not None:
            np.copyto(self.Tr, t)
            self.do_inject()

    def setRT(self, ox: float, oy: float, oz: float, t: InnerModelVector):
        self.Rx.update(ox)
        self.Ry.update(oy)
        self.Rz.update(oz)
        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto (self.R, R)
        np.copyto(self.Tr, t)
        self.do_inject()

    def getTr(self) -> 'InnerModelVector':
        return self.Tr

    def getR(self) -> 'InnerModelMatrix':
        return self.R

    def getRx(self) -> 'Rot3DOX':
        return self.Rx

    def getRy(self) -> 'Rot3DOY':
        return self.Ry

    def getRz(self) -> 'Rot3DOZ':
        return self.Rz

    def getRxValue(self) -> float:
        return self.Rx.getAlpha()

    def getRyValue(self) -> float:
        return self.Ry.getAlpha()

    def getRzValue(self) -> float:
        return self.Rz.getAlpha()

    def invert(self) -> 'InnerModelRTMat':
        r = InnerModelRTMat(self.shape)
        np.copyto(r.R, self.R.transpose())
        _t = r.R.dot(self.Tr)*(-1)
        np.copyto(r.Tr, _t)
        r.do_inject()
        return r

    def invertR(self) -> 'InnerModelMatrix':
        r = InnerModelMatrix((4,4))
        np.copyto (r[:3,:3], self.R.transpose())
        r[3][3] = 1.
        return r

    def serializeAsString(self) -> str:
        s = str(self.Tr.x()) + ' ' + str(self.Tr.y()) + ' ' + str(self.Tr.z()) + \
            str(self.Rx.getAlpha()) + ' ' + str(self.Ry.getAlpha()) + str(self.Rz.getAlpha())
        return s

class InnerModelRTCMat(InnerModelMatrix):
    @staticmethod
    def getInnerModelRTCMat (ox: float, oy: float, oz: float, x: float, y: float, z: float):
        mat = InnerModelRTCMat((4,4))
        mat.Rx = Rot3DCOX.getRot3DCOX(alpha=ox)
        mat.Ry = Rot3DCOY.getRot3DCOY(alpha=oy)
        mat.Rz = Rot3DCOZ.getRot3DCOZ(alpha=oz)
        R = mat.Rx.dot(mat.Ry.dot(mat.Rz))
        np.copyto(mat.R, R)
        mat.Tr = InnerModelVector.vec3d (x, y, z)

    def __new__(cls, *args, **kwargs):
        return super(InnerModelRTCMat, cls).__new__(cls, *args, **kwargs)

    def __init__(self, *args, **kwargs):
        self.Rx = Rot3DCOX.getRot3DCOX(alpha=0)
        self.Ry = Rot3DCOY.getRot3DCOY(alpha=0)
        self.Rz = Rot3DCOZ.getRot3DCOZ(alpha=0)
        self.R = InnerModelMatrix.identity(3)
        self.Tr = InnerModelVector.vec3d(0, 0, 0)
        self.do_inject()

    def init(self, ox: float, oy: float, oz: float, t: InnerModelVector):
        np.copyto(self.Tr, t)
        self.Rx.update(ox)
        self.Ry.update(oy)
        self.Rz.update(oz)
        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto(self.R, R)
        self.do_inject()

    def do_inject (self):
        np.copyto (self[:3, :3], self.R)
        np.copyto (self[:3, 3], self.Tr)
        self[3][0] = 0.
        self[3][1] = 0.
        self[3][2] = 0.
        self[3][3] = 1.

    def set(self, ox: float, oy: float, oz: float, x: float, y: float, z: float):
        self.Tr[0] = x
        self.Tr[1] = y
        self.Tr[2] = z

        self.Rx.update(ox)
        self.Ry.update(oy)
        self.Rz.update(oz)

        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto(self.R, R)
        self.do_inject()

    def setR(self, ox: float, oy: float, oz: float):
        self.Rx.update(ox)
        self.Ry.update(oy)
        self.Rz.update(oz)
        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto(self.R, R)
        self.do_inject()

    def setRX(self, ox: float):
        self.Rx.update(ox)
        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto(self.R, R)
        self.do_inject()

    def setRY(self, oy: float):
        self.Ry.update(oy)
        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto(self.R, R)
        self.do_inject()

    def setRZ(self, oz: float):
        self.Rz.update(oz)
        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto(self.R, R)
        self.do_inject()

    def setTr(self, t: InnerModelVector = None):
        if t is not None:
            np.copyto(self.Tr, t)
            self.do_inject()

    def setRT(self, ox: float, oy: float, oz: float, t: InnerModelVector):
        self.Rx.update(ox)
        self.Ry.update(oy)
        self.Rz.update(oz)
        R = self.Rx.dot(self.Ry.dot(self.Rz))
        np.copyto(self.R, R)
        np.copyto(self.Tr, t)
        self.do_inject()

    def getTr(self) -> 'InnerModelVector':
        return self.Tr

    def getR(self) -> 'InnerModelMatrix':
        return self.R

    def getRx(self) -> 'Rot3DOX':
        return self.Rx

    def getRy(self) -> 'Rot3DOY':
        return self.Ry

    def getRz(self) -> 'Rot3DOZ':
        return self.Rz

    def getRxValue(self) -> float:
        return self.Rx.getAlpha()

    def getRyValue(self) -> float:
        return self.Ry.getAlpha()

    def getRzValue(self) -> float:
        return self.Rz.getAlpha()

    def invert(self) -> 'InnerModelRTCMat':
        r = InnerModelRTCMat(self.shape)
        np.copyto(r.R, self.R.transpose())
        _t = r.R.dot(self.Tr)*(-1)
        np.copyto(r.Tr, _t)
        r.do_inject()
        return r

    def invertR(self) -> 'InnerModelMatrix':
        r = InnerModelMatrix((4,4))
        np.copyto (r[:3,:3], self.R.transpose())
        r[3][3] = 1.
        return r

    def serializeAsString(self) -> str:
        s = str(self.Tr.x()) + ' ' + str(self.Tr.y()) + ' ' + str(self.Tr.z()) + \
            str(self.Rx.getAlpha()) + ' ' + str(self.Ry.getAlpha()) + str(self.Rz.getAlpha())
        return s
