import unittest
import sys

sys.path.append('.')
sys.path.append('..')

from innermodel import InnerModel

im = InnerModel (xmlFilePath='/home/robocomp/robocomp/files/innermodel/simplesimpleworld.xml')

im.printTree()
