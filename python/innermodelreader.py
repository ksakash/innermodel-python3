#!/usr/bin/env python3

import xml.etree.ElementTree as ET
from innermodel import InnerModel

class InnerModelReader (object):
    def __init__ (self):
        self.validAttr = dict()
        self.intNodeAttributes()

    def intNodeAttributes (self):
        attrList = []
        self.validAttr["innermodel"] = attrList

        attrList = ["id", "rx", "ry", "rz", "engine", "mass"]
        self.validAttr["rotation"] = attrList

        attrList = ["id", "tx", "ty", "tz", "engine", "mass"]
        self.validAttr["translation"] = attrList

        attrList = ["id", "tx", "ty", "tz", "rx", "ry", "rz", "engine", "mass"]
        self.validAttr["transform"] = attrList

        attrList = ["id", "lx", "ly", "lz", "hx", "hy", "hz", "tx", "ty", "tz", "rx", "ry", "rz",
                    "min", "max", "port", "axis", "home"]
        self.validAttr["joint"] = attrList

        attrList = ["id", "type", "nx", "ny", "nz", "min", "max", "port"]
        self.validAttr["touchsensor"] = attrList

        attrList = ["id", "min", "max", "position", "offset", "port", "axis", "home"]
        self.validAttr["prismaticjoint"] = attrList

        attrList = ["id", "tx", "ty", "tz", "rx", "ry", "rz", "port", "noise", "collide"]
        self.validAttr["omnirobot"] = attrList

        attrList = ["id", "width", "height", "focal", "noise", "port", "ifconfig"]
        self.validAttr["rgbd"] = attrList

        attrList = ["id", "port"]
        self.validAttr["imu"] = attrList

        attrList = ["id", "port", "min", "max", "angle", "measures", "ifconfig"]
        self.validAttr["laser"] = attrList

        attrList = ["id", "width", "height", "focal"]
        self.validAttr["camera"] = attrList

        attrList =["id", "file", "scale", "render", "tx", "ty", "tz", "rx", "ry", "rz", "collide"]
        self.validAttr["mesh"] = attrList

        attrList = ["id"]
        self.validAttr["pointcloud"] = attrList

        attrList = ["id", "texture", "repeat", "size", "nx", "ny", "nz", "px", "py", "pz", "collide"]
        self.validAttr["plane"] = attrList

        attrList = ["id", "texture", "repeat", "size", "nx", "ny", "nz",
                    "px", "py", "pz", "collide", "port"]
        self.validAttr["display"] = attrList

        attrList = ["path"]
        self.validAttr["include"] = attrList

        attrList = ["id", "length", "width", "lengthx", "widthx", "lengthy", "widthy", "lengthz", "widthz"]
        self.validAttr["axes"] = attrList

    def load (self, file: str, model: 'InnerModel') -> bool:
        tree = ET.parse (file)
        root = tree.getroot()
        model = InnerModel()
        InnerModelReader.recursive (root, model, model.root)

    @staticmethod
    def include (file: str, model: 'InnerModel', node: 'InnerModelNode') -> bool:
        pass

    @staticmethod
    def recursive (parentDomNode: 'ET', model: 'InnerModel', imNode: 'InnerModelNode'):
        pass

    @staticmethod
    def getValidNodeAttributes ():
        pass

