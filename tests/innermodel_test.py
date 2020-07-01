import unittest
import sys

sys.path.append('../innermodel_python3')

from innermodel import InnerModel

im = InnerModel (xmlFilePath='test.xml')
print (im.save ('test2.xml'))
