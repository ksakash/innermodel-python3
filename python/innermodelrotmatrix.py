import math
import numpy as np
from innermodelmatrix import InnerModelMatrix

# Base class for all the rotation matrices
class Rot3DOnAxis(InnerModelMatrix):

    @staticmethod
    def getRot3DOnAxis(alpha: float) -> 'Rot3DOnAxis':
        x = Rot3DOnAxis((3,3))
        x.ang = alpha
        return x

    def __new__(cls, *args, **kwargs):
        return super(Rot3DOnAxis, cls).__new__(cls, *args, **kwargs)

    def __init__(self, *args, **kwargs):
        self.ang = 0

    def update (self, alpha: float):
        pass

    def getAlpha (self) -> float:
        return self.ang

# 3D Rotation matrix around the x axis CCW
class Rot3DOX(Rot3DOnAxis):
    @staticmethod
    def getRot3DOX(alpha: float = 0, m : 'Rot3DOX' = None) -> 'Rot3DOX':
        if m is not None:
            alpha = m.getAlpha()
            x = Rot3DOX((3,3))
            np.copyto(x, m)
            x.update (alpha)
            return x

        x = Rot3DOX((3,3))
        x.update(alpha)
        x[0][0] = 1
        x[0][1] = 0
        x[0][2] = 0
        x[1][0] = 0
        x[2][0] = 0
        return x

    def update (self, alpha: float):
        self[1][1] = math.cos(alpha)
        self[1][2] = -math.sin(alpha)
        self[2][1] = math.sin(alpha)
        self[2][2] = math.cos(alpha)
        self.ang = alpha

# 3D Rotation matrix around the x axis CW
class Rot3DCOX(Rot3DOnAxis):
    @staticmethod
    def getRot3DCOX(alpha: float = 0, m : 'Rot3DCOX'= None) -> 'Rot3DCOX':
        if m is not None:
            alpha = m.getAlpha()
            x = Rot3DCOX((3,3))
            np.copyto(x, m)
            x.update (alpha)
            return x

        x = Rot3DCOX((3,3))
        x.update(alpha)
        x[0][0] = 1
        x[0][1] = 0
        x[0][2] = 0
        x[1][0] = 0
        x[2][0] = 0
        return x

    def update (self, alpha: float):
        self[1][1] = math.cos(alpha)
        self[1][2] = math.sin(alpha)
        self[2][1] = -math.sin(alpha)
        self[2][2] = math.cos(alpha)
        self.ang = alpha

# 3D Rotation matrix around the y axis CCW
class Rot3DOY(Rot3DOnAxis):
    @staticmethod
    def getRot3DOY(alpha: float = 0, m : 'Rot3DOY' = None) -> 'Rot3DOY':
        if m is not None:
            alpha = m.getAlpha()
            y = Rot3DOY((3,3))
            np.copyto(y, m)
            y.update (alpha)
            return y

        y = Rot3DOY((3,3))
        y.update(alpha)
        y[0][1] = 0
        y[1][0] = 0
        y[1][1] = 1
        y[1][2] = 0
        y[2][1] = 0
        return y

    def update (self, alpha: float):
        self[0][0] = math.cos(alpha)
        self[0][2] = math.sin(alpha)
        self[2][0] = -math.sin(alpha)
        self[2][2] = math.cos(alpha)
        self.ang = alpha

# 3D Rotation matrix around the x axis CW
class Rot3DCOY(Rot3DOnAxis):
    @staticmethod
    def getRot3DCOY(alpha: float = 0, m : 'Rot3DCOY' = None) -> 'Rot3DCOY':
        if m is not None:
            alpha = m.getAlpha()
            y = Rot3DCOY((3,3))
            np.copyto(y, m)
            y.update (alpha)
            return y

        y = Rot3DCOY((3,3))
        y.update(alpha)
        y[0][1] = 0
        y[1][0] = 0
        y[1][1] = 1
        y[1][2] = 0
        y[2][1] = 0
        return y

    def update (self, alpha: float):
        self[0][0] = math.cos(alpha)
        self[0][2] = -math.sin(alpha)
        self[2][0] = math.sin(alpha)
        self[2][2] = math.cos(alpha)
        self.ang = alpha

# 3D Rotation matrix around the z axis CCW
class Rot3DOZ(Rot3DOnAxis):
    @staticmethod
    def getRot3DOZ(alpha: float = 0, m : 'Rot3DOZ' = None) -> 'Rot3DOZ':
        if m is not None:
            alpha = m.getAlpha()
            z = Rot3DOZ((3,3))
            np.copyto(z, m)
            z.update (alpha)
            return z

        z = Rot3DOZ((3,3))
        z.update(alpha)
        z[0][2] = 0
        z[1][2] = 0
        z[2][0] = 0
        z[2][1] = 0
        z[2][2] = 1
        return z

    def update (self, alpha: float):
        self[0][0] = math.cos(alpha)
        self[0][1] = -math.sin(alpha)
        self[1][0] = math.sin(alpha)
        self[1][1] = math.cos(alpha)
        self.ang = alpha

# 3D Rotation matrix around the x axis CW
class Rot3DCOZ(Rot3DOnAxis):
    @staticmethod
    def getRot3DCOZ(alpha: float = 0, m : 'Rot3DCOZ' = None) -> 'Rot3DCOZ':
        if m is not None:
            alpha = m.getAlpha()
            y = Rot3DCOZ((3,3))
            np.copyto(y, m)
            y.update (alpha)
            return y

        z = Rot3DCOZ((3,3))
        z.update(alpha)
        z[0][2] = 0
        z[1][2] = 0
        z[2][0] = 0
        z[2][1] = 0
        z[2][2] = 1
        return z

    def update (self, alpha: float):
        self[0][0] = math.cos(alpha)
        self[0][1] = math.sin(alpha)
        self[1][0] = -math.sin(alpha)
        self[1][1] = math.cos(alpha)
        self.ang = alpha

# Rotation matrix CCW
class Rot3D(InnerModelMatrix):
    @staticmethod
    def getRot3D (ox: float = 0, oy: float = 0, oz: float = 0, ex = None) -> 'Rot3D':
        if ex is not None:
            mat = Rot3D(ex.shape)
            np.copyto (mat, ex)

            np.copyto(mat.RX, ex.RX)
            np.copyto(mat.RY, ex.RY)
            np.copyto(mat.RZ, ex.RZ)

            return mat
        else:
            mat = Rot3D((3,3))
            mat.RX = Rot3DOX.getRot3DOX(ox)
            mat.RY = Rot3DOY.getRot3DOY(oy)
            mat.RZ = Rot3DOZ.getRot3DOZ(oz)
            _m  = mat.RX*mat.RY*mat.RZ
            np.copyto(mat, _m)
            return mat

    def __new__(cls, *args, **kwargs):
        return super(Rot3D, cls).__new__(cls, *args, **kwargs)

    def __init__(self, *args, **kwargs):
        self.RX = None
        self.RY = None
        self.RZ = None

    def update (self, ox: float, oy: float, oz: float):
        self.RX.update(ox)
        self.RY.update(oy)
        self.RZ.update(oz)
        _m  = self.RX*self.RY*self.RZ
        np.copyto(self, _m)

# Rotation matrix CW
class Rot3DC(InnerModelMatrix):
    @staticmethod
    def getRot3DC (ox: float = 0, oy: float = 0, oz: float = 0, XCW: bool = True,
                YCW: bool = True, ZCW: bool = True, ex = None) -> 'Rot3DC':
        if ex is not None:
            mat = Rot3DC(ex.shape)
            np.copyto (mat, ex)

            np.copyto (mat.RX, ex.RX)
            np.copyto (mat.RY, ex.RY)
            np.copyto (mat.RZ, ex.RZ)

            return mat
        else:
            mat = Rot3DC((3,3))
            mat.RX = Rot3DOX.getRot3DOX(ox)
            mat.RY = Rot3DOY.getRot3DOY(oy)
            mat.RZ = Rot3DOZ.getRot3DOZ(oz)
            _m  = mat.RX*mat.RY*mat.RZ
            np.copyto(mat, _m)
            return mat

    def __new__(cls, *args, **kwargs):
        return super(Rot3DC, cls).__new__(cls, *args, **kwargs)

    def __init__(self, *args, **kwargs):
        self.RX = None
        self.RY = None
        self.RZ = None

    def update (self, ox: float, oy: float, oz: float):
        self.RX.update(ox)
        self.RY.update(oy)
        self.RZ.update(oz)
        _m  = self.RX*self.RY*self.RZ
        np.copyto(self, _m)

class Rot2D(InnerModelMatrix):
    @staticmethod
    def getRot2D(alpha: float) -> 'Rot2D':
        mat = Rot2D((2,2))
        mat[0][0] = math.cos(alpha)
        mat[0][1] = math.sin(alpha)
        mat[1][0] = -math.sin(alpha)
        mat[1][1] = math.cos(alpha)
        return mat

    def update (self, alpha: float):
        self[0][0] = math.cos(alpha)
        self[0][1] = math.sin(alpha)
        self[1][0] = -math.sin(alpha)
        self[1][1] = math.cos(alpha)

class Rot2DC(InnerModelMatrix):
    @staticmethod
    def getRot2DC(alpha: float) -> 'Rot2DC':
        mat = Rot2DC((2,2))
        mat[0][0] = math.cos(alpha)
        mat[0][1] = -math.sin(alpha)
        mat[1][0] = math.sin(alpha)
        mat[1][1] = math.cos(alpha)
        return mat

    def update (self, alpha: float):
        self[0][0] = math.cos(alpha)
        self[0][1] = -math.sin(alpha)
        self[1][0] = math.sin(alpha)
        self[1][1] = math.cos(alpha)
