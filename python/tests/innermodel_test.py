import unittest
import sys

sys.path.append('.')
sys.path.append('..')

from innermodel import InnerModel

im = InnerModel (xmlFilePath='test.xml')
print (im.save ('test2.xml'))
