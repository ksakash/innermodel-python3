import unittest

import sys
sys.path.append('.')
sys.path.append('..')

from innermodelvector import InnerModelVector

class TestVectorMethods(unittest.TestCase):

    def test_3d(self):
        a = InnerModelVector.vec3d(1., 2., 3.)
        b = InnerModelVector.vec3d(3., 2., 1.)
        assert(a[0][0] + b[0][0] == 1. + 3.)
        assert(a[1][0] + b[1][0] == 2. + 2.)
        assert(a[2][0] + b[2][0] == 3. + 1.)


if __name__ == '__main__':
    unittest.main()