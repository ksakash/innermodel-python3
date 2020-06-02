import sys

sys.path.append ('.')
sys.path.append ('..')

from innermodel import InnerModel
from innermodelviewer import InnerModelViewer

im = InnerModel (xmlFilePath='test.xml')
imv = InnerModelViewer (innermodel=im)
imv.render()
