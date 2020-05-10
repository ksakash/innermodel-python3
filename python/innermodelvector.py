# -*- coding: utf-8 -*-

import numpy as np

class InnerModelVector(np.ndarray):

    @staticmethod
    def vec3d(x: float, y: float, z: float) -> 'InnerModelVector':
        ret = InnerModelVector((3,))
        ret[0] = x
        ret[1] = y
        ret[2] = z
        return ret

    @staticmethod
    def vec6d(x: float, y: float, z: float, rx: float, ry: float, rz: float) -> 'InnerModelVector':
        ret = InnerModelVector((6,))
        ret[0] = x
        ret[1] = y
        ret[2] = z
        ret[3] = rx
        ret[4] = ry
        ret[5] = rz
        return ret

    def set(self, value: float):
        self.fill(value)

    def subvector(self, firstIndex: int, lastIndex: int) -> 'InnerModelVector':
        assert(firstIndex >= 0)
        assert(firstIndex <= lastIndex)
        return self[firstIndex : lastIndex]

    def scalar_division(self, value: float) -> 'InnerModelVector':
        assert (value != 0)
        return (self/value)

    def scalar_multiplication(self, value: float) -> 'InnerModelVector':
        return self*value

    def inject(self, other: 'InnerModelVector', offset: int):
        assert(other.shape[0] + offset <= self.shape[0])
        size = other.shape[0]
        self[offset:offset+size] = other

    def point_product(self, other: 'InnerModelVector') -> 'InnerModelVector':
        return self*other

    def normalized(self) -> 'InnerModelVector':
        return self.scalar_division(self.norm2())

    def cross_product(self, other: 'InnerModelVector') -> 'InnerModelVector':
        return np.cross(self, other)

    def equals(self, other: 'InnerModelVector') -> bool:
        return np.array_equal(self, other)

    def is_zero(self) -> bool:
        size = self.shape[0]
        return np.array_equal(self, np.zeros(size))

    def to_homogeneous_coordinates(self) -> 'InnerModelVector':
        return np.append(self, 1.0)

    def from_homogeneous_coordinates(self) -> 'InnerModelVector':
        size = self.shape[0]
        last = self[size-1]
        self = np.resize(self, size-1)
        assert(last != 0)
        return (self/last)

    def norm2(self) -> float:
        return np.linalg.norm(self)

    def min(self) -> [float, int]:
        _i = np.argmin(self)
        _m = self[_i]
        return [_m, _i]

    def max(self) -> [float, int]:
        _i = np.argmax(self)
        _m = self[_i]
        return [_m, _i]

    def min_abs(self) -> [float, int]:
        _abs = np.absolute(self)
        _i = np.argmin(_abs)
        _m = _abs[_i]
        return [_m, _i]

    def max_abs(self) -> [float, int]:
        _abs = np.absolute(self)
        _i = np.argmax(_abs)
        _m = _abs[_i]
        return [_m, _i]

    def mean(self) -> float:
        return np.mean(self)

    def variance(self) -> float:
        return np.var(self)

    def dot_product(self, other: 'InnerModelVector') -> float:
        return np.dot(self, other)

    def x(self):
        assert(self.shape[0] >= 3)
        return self[0]

    def y(self):
        assert(self.shape[0] >= 3)
        return self[1]

    def z(self):
        assert(self.shape[0] >= 3)
        return self[2]

    def rx(self):
        assert(self.shape[0] >= 6)
        return self[3]

    def ry(self):
        assert(self.shape[0] >= 6)
        return self[4]

    def rz(self):
        assert(self.shape[0] >= 6)
        return self[5]


