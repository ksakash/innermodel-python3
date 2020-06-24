import unittest
import sys

sys.path.append('.')
sys.path.append('..')

from innermodel import InnerModel

im = InnerModel (xmlFilePath='test.xml')

im.printTree()

fake_im = im.copy()
print ('----------')
fake_im.printTree ()
