import math
import numpy as np
from innermodelvector import InnerModelVector
from scipy.spatial.transform import Rotation as R

class InnerModelMatrix(np.ndarray):

    # empty matrix
    # m: no. of rows
    # n: no. of cols
    @staticmethod
    def Mat (m: int, n: int) -> 'InnerModelMatrix':
        matrix = InnerModelMatrix((m,n))
        return matrix

    # diagonal matrix
    @staticmethod
    def makeDiagonal (v: 'InnerModelVector') -> 'InnerModelMatrix':
        x = np.diag (v)
        y = InnerModelMatrix(x.shape)
        np.copyto(y,x)
        return y

    # m: no. of rows and cols
    @staticmethod
    def identity (m: int) -> 'InnerModelMatrix':
        y = InnerModelMatrix((m,m))
        np.copyto(y, np.identity(m))
        return y

    # keep the diagonal values only
    @staticmethod
    def diagonalMat (m: 'InnerModelMatrix') -> 'InnerModelMatrix':
        assert (m.shape[0] == m.shape[1])
        d = m.diagonal()
        x = np.diag(d)
        y = InnerModelMatrix(m.shape)
        np.copyto(y, x)
        return y

    @staticmethod
    def ones (m: int, n: int) -> 'InnerModelMatrix':
        x = np.ones ((m,n))
        y = InnerModelMatrix((m,n))
        np.copyto(y,x)
        return y

    # create a matrix of the given size with random numbers between 0 & 1
    @staticmethod
    def random (m: int, n: int) -> 'InnerModelMatrix':
        x = np.random.rand (m,n)
        y = InnerModelMatrix((m,n))
        np.copyto(y,x)
        return y

    @staticmethod
    def zeroes (m: int, n: int) -> 'InnerModelMatrix':
        x = np.zeros ((m,n))
        y = InnerModelMatrix((m,n))
        np.copyto(y,x)
        return y

    @staticmethod
    def equalSize (first: 'InnerModelMatrix', second: 'InnerModelMatrix') -> bool:
        return (first.shape[0] == second.shape[0] and first.shape[1] == second.shape[1])

    # set the value at all the indices
    def set (self, value: float):
        self.fill(value)

    def copy (self) -> 'InnerModelMatrix':
        mat = InnerModelMatrix(self.shape)
        np.copyto(mat, self)
        return mat

    def inject (self, matrix: 'InnerModelMatrix', roff: float, coff: float) -> 'InnerModelMatrix':
        rows = matrix.shape[0]
        cols = matrix.shape[1]
        assert (rows+roff <= self.shape[0] and cols+coff <= self.shape[1])
        self[roff:roff+rows, coff:coff+cols] = matrix
        mat = InnerModelMatrix(self.shape)
        np.copyto(mat, self)
        return mat

    # change the diagonal
    # elements to the given value
    def fillDiagonal (self, value: float):
        np.fill_diagonal(self, value)

    def getDiagonal (self) -> 'InnerModelVector':
        x = self.diagonal()
        y = InnerModelVector((x.shape[0]))
        np.copyto(y, x)
        return y

    def getTranspose (self) -> 'InnerModelMatrix':
        return self.transpose()

    def determinant (self) -> float:
        assert (self.shape[0] == self.shape[1])
        return np.linalg.det(self)

    def trace (self) -> float:
        assert (self.shape[0] == self.shape[1])
        m = np.array(self)
        return np.trace(m)

    # pseudo-inverse # TODO
    def invert (self) -> 'InnerModelMatrix':
        assert(self.isSquare())
        if (self.determinant() != 0):
            m = np.linalg.inv(self)
            mat = InnerModelMatrix(m.shape)
            np.copyto(mat, m)
            return mat
        else:
            m = np.linalg.pinv(self)
            mat = InnerModelMatrix(m.shape)
            np.copyto(mat, m)
            return mat

    def fillOnes (self):
        self.set(1)

    def makeUnitModulus (self) -> 'InnerModelMatrix':
        assert(self.isColVector())
        mod = self.vectorNormL2()
        if (mod > 0):
            self = self/mod
        y = InnerModelMatrix(self.shape)
        np.copyto(y, self)
        return y

    # make the current matrix identity
    def makeIdentity (self):
        assert(self.shape[0] == self.shape[1])
        x = np.identity(self.shape[0])
        np.copyto(self, x)

    def isSquare (self, other: 'InnerModelMatrix' = None) -> bool:
        if other is not None:
            return (other.shape[0] == other.shape[1])
        return (self.shape[0] == self.shape[1])

    def is3ColumnVector (self, other: 'InnerModelMatrix' = None) -> bool:
        if other is not None:
            return (other.shape[0] == 3 and other.shape[1] == 1)
        return (self.shape[0] == 3 and self.shape[1] == 1)

    def canAllocateRotationMatrix (self) -> bool:
        return (self.shape[0] >= 3 and self.shape[1] >= 3)

    def isColVector (self) -> bool:
        return (self.shape[0] > 0 and self.shape[1] == 1)

    def isEmpty (self) -> bool:
        return not (self.size > 0)

    def minDim (self, other: 'InnerModelMatrix' = None) -> int:
        if (other is not None):
            return min (other.shape[0], other.shape[1])
        return min (self.shape[0], self.shape[1])

    def maxDim (self, other: 'InnerModelMatrix' = None) -> int:
        if (other is not None):
            return max (other.shape[0], other.shape[1])
        return max (self.shape[0], self.shape[1])

    def sqrt (self) -> 'InnerModelMatrix':
        _abs = np.absolute (self)
        return np.sqrt (_abs)

    # to check if the matrix is positive definite
    def isPosDef (self) -> bool:
        assert (self.isSquare())
        return np.all(np.linalg.eigvals(self) > 0)

    def cholesky (self) -> 'InnerModelMatrix':
        assert (self.isPosDef())
        return np.linalg.cholesky(self)

    def eigenValsVectors (self) -> ('InnerModelVector', 'InnerModelMatrix'):
        assert (self.isSquare())
        w, V = np.linalg.eig (self)
        y = InnerModelVector(w.shape)
        np.copyto (y, w)
        return (y, V)

    def SVD (self) -> ('InnerModelMatrix', 'InnerModelMatrix', 'InnerModelMatrix'):
        U, _S, V_T = np.linalg.svd (self, full_matrices=True)
        S = InnerModelMatrix.zeroes (self.shape[0], self.shape[1])
        size = _S.shape[0]
        for i in range (size):
            S[i][i] = _S[i]
        return (U, S, V_T.getTranspose())

    # make definite positive
    def makeDefPos (self) -> 'InnerModelMatrix':
        (w, V) = self.eigenValsVectors()
        size = w.shape[0]
        for i in range(size):
            if w[i] <= 0:
                w[i] = 0.000001
        DD = InnerModelMatrix.makeDiagonal(w)
        return (V*DD*V.getTranspose())

    # TODO
    def matSqrt (self) -> 'InnerModelMatrix':
        assert (self.isSquare())
        (_, V) = self.eigenValsVectors()
        V_I = V.invert()
        _D = V_I * self * V
        D = InnerModelMatrix.zeroes(_D.shape[0], _D.shape[1])
        size = D.shape[0]
        for i in range (size):
            D[i][i] = math.sqrt(abs(_D[i][i]))
        R = V*D*V_I
        return R

    def vectorNormL2 (self) -> float:
        sum = 0
        size = self.shape[0]

        for i in range(size):
            sum += np.linalg.norm(self[i])**2
        return sum

    def toVector (self) -> 'InnerModelVector':
        assert (self.shape[0] > 0 and self.shape[1] == 1)
        rows = self.shape[0]
        y = InnerModelVector((rows,))
        np.copyto (y, self[:, 0])
        return y

    # Eulers angles from rotation matrix in radians
    def extractAnglesR(self) -> 'InnerModelMatrix':
        assert (self.shape == (3,3))
        r = R.from_matrix(self)
        _v = r.as_euler('xyz')
        v = InnerModelMatrix(_v.shape)
        np.copyto(v, _v)
        return v

    # Choosing the one which has lower norm
    def extractAnglesR_min (self) -> 'InnerModelVector':
        r = self.extractAnglesR()
        if (r.shape[0] == 1):
            return InnerModelVector.vec3d(r[0], r[1], r[2])
        v1 = InnerModelVector((3,))
        np.copyto(v1, r[0])
        v2 = InnerModelVector((3,))
        np.copyto(v2, r[1])
        if (np.linalg.norm(v1) < np.linalg.norm(v2)):
            return v1
        return v2

    def setCol (self, col: int, vector: 'InnerModelVector'):
        assert (self.shape[0] == vector.shape[0])
        np.copyto (self[:,col], vector)

    def getCol (self, col: int) -> 'InnerModelVector':
        y = InnerModelVector ((self.shape[0],))
        np.copyto (y, self[:,col])
        return y

    def setRow (self, row: int, vector: 'InnerModelVector'):
        assert (self.shape[1] == vector.shape[0])
        np.copyto (self[row,:], vector)

    def getRow (self, row: int) -> 'InnerModelVector':
        y = InnerModelVector ((self.shape[1],))
        np.copyto (y, self[row,:])
        return y

    def getSubmatrix (self, firstRow: int, lastRow: int, firstCol: int, lastCol: int) -> 'InnerModelMatrix':
        return self[firstRow:lastRow+1, firstCol:lastCol+1]
