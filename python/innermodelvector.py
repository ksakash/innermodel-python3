# -*- coding: utf-8 -*-

import numpy as np

class InnerModelVector(np.ndarray):

    @staticmethod
    def vec3d(x: float, y: float, z: float) -> 'InnerModelVector':
        ret = InnerModelVector((3, 1))
        ret[0] = x
        ret[1] = y
        ret[2] = z
        return ret

    @staticmethod
    def vec6d(x: float, y: float, z: float, rx: float, ry: float, rz: float) -> 'InnerModelVector':
        ret = InnerModelVector((3, 1))
        ret[0] = x
        ret[1] = y
        ret[2] = z
        ret[3] = x
        ret[4] = y
        ret[5] = z
        return ret

    def set(self, value: float):
        pass

    def subvector(self, firstIndex: int, lastIndex: int) -> 'InnerModelVector':
        pass

    def scalar_division(self, value: float) -> 'InnerModelVector':
        pass

    def scalar_multiplication(self, value: float) -> 'InnerModelVector':
        pass

    def inject(self, other: 'InnerModelVector', offset: int):
        pass

    def point_product(self, other: 'InnerModelVector') -> 'InnerModelVector':
        pass

    def normalized(self) -> 'InnerModelVector':
        return self.scalar_division(self.norm2())

    def cross_product(self, other: 'InnerModelVector') -> 'InnerModelVector':
        pass
    
    def equals(self, other: 'InnerModelVector') -> bool:
        pass

    def is_zero(self) -> bool:
        pass
    
    def to_homogeneous_coordinates(self) -> 'InnerModelVector':
        pass

    def from_homogeneous_coordinates(self) -> 'InnerModelVector':
        pass

    def norm2(self) -> float:
        pass

    def min(self) -> [float, int]:
        pass

    def max(self) -> [float, int]:
        pass
    
    def min_abs(self) -> [float, int]:
        pass

    def max_abs(self) -> [float, int]:
        pass

    def mean(self) -> float:
        pass

    def variance(self) -> float:
        pass

    def dot_product(self, other: 'InnerModelVector') -> float:
        pass

    def x(self):
        assert(self.shape[0] >= 0 and self.shape[1] >= 3)
        return self[0][0]

    def y(self):
        assert(self.shape[0] >= 0 and self.shape[1] >= 3)
        return self[0][1]

    def z(self):
        assert(self.shape[0] >= 0 and self.shape[1] >= 3)
        return self[0][2]

    def rx(self):
        assert(self.shape[0] >= 0 and self.shape[1] >= 6)
        return self[0][3+0]

    def ry(self):
        assert(self.shape[0] >= 0 and self.shape[1] >= 6)
        return self[0][3+1]

    def rz(self):
        assert(self.shape[0] >= 0 and self.shape[1] >= 6)
        return self[0][3+2]


