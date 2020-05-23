import unittest
import sys

sys.path.append('.')
sys.path.append('..')

from innermodel import InnerModel

im = InnerModel (xmlFilePath='../thefile.xml')

# def DFS (root):
#     print (root.id)
#     for child in root.children:
#         DFS (child)

# DFS (im.root)

im.printTree()
