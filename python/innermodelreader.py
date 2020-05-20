#!/usr/bin/env python3

import xml.etree.ElementTree as ET
from innermodel import InnerModel
from innermodelnode import InnerModelNode

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

    @staticmethod
    def load (file: str, model: 'InnerModel') -> bool:
        tree = ET.parse (file)
        root = tree.getroot()
        model = InnerModel()
        InnerModelReader.recursive (root, model, model.root)

    @staticmethod
    def include (file: str, model: 'InnerModel', node: 'InnerModelNode') -> bool:
        tree = ET.parse (file)
        root = tree.getroot ()
        model = InnerModel ()
        InnerModelReader.recursive (root, model, model.root)
        return True

    # need a bit modification for the tags having dimensions
    @staticmethod
    def getClass (model, node, tag, attr):
            if tag.lower() == 'rotation':
                tr = model.newTransform (id=attr['id'], engine=attr['engine'], parent=node,
                                    tx=0, ty=0, tz=0, rx=attr['rx'], ry=attr['ry'], rz=attr['rz'],
                                    mass=attr['mass'])
                tr.gui_translation = False
                return tr
            elif tag.lower() == 'translation':
                tr = model.newTransform (id=attr['id'], engine=attr['engine'], parent=node,
                                    tx=attr['tx'], ty=attr['ty'], tz=attr['tz'], rx=0, ry=0, rz=0,
                                    mass=attr['mass'])
                tr.gui_rotation = False
                return tr
            elif tag.lower() == 'transform':
                tr = model.newTransform (id=attr['id'], engine=attr['engine'], parent=node,
                                    tx=attr['tx'], ty=attr['ty'], tz=attr['tz'], rx=attr['rx'],
                                    ry=attr['ry'], rz=attr['rz'], mass=attr['mass'])
                return tr
            elif tag.lower() == 'touchsensor':
                ts = model.newTouchSensor (id=attr['id'], stype=attr['type'], nx=attr['nx'],
                                           ny=attr['ny'], nz=attr['nz'], min=attr['min'],
                                           max=attr['max'], port=attr['port'], parent=node)
                return ts
            elif tag.lower() == 'joint':
                jr = model.newJoint (id=attr[id], lx=attr['lx'], ly=attr['ly'], lz=attr['lz'],
                                    hx=attr['hx'], hy=attr['hy'], hz=attr['hz'], tx=attr['tx'],
                                    ty=attr['ty'], tz=attr['tz'], rx=attr['rx'], ry=attr['ry'],
                                    rz=attr['rz'], min=attr['min'], max=attr['max'], port=attr['port'],
                                    axis=attr['axis'], home=attr['home'], parent=node)
                return jr
            elif tag.lower() == 'prismaticjoint':
                jr = model.newPrismaticJoint (id=attr['id'], parent=node, min=attr['min'], max=attr['max'],
                                              position=attr['position'], offset=attr['offset'],
                                              port=attr['port'], axis=attr['axis'], home=attr['home'])
                return jr
            elif tag.lower() == 'differentialrobot':
                dr = model.newDifferentialRobot (id=attr['id'], tx=attr['tx'], ty=attr['ty'],
                                                tz=attr['tz'], rx=attr['rx'], ry=attr['ry'],
                                                rz=attr['rz'], port=attr['port'], noise=attr['noise'],
                                                collide=attr['collide'], parent=node)
                return dr
            elif tag.lower() == 'omnirobot':
                om = model.newOmniRobot (id=attr['id'], tx=attr['tx'], ty=attr['ty'], tz=attr['tz'],
                                         rx=attr['rx'], ry=attr['ry'], rz=attr['rz'], port=attr['port'],
                                         noise=attr['noise'], collide=attr['collide'], parent=node)
                return om
            elif tag.lower() == 'camera':
                cam = model.newCamera (id=attr['id'], width=attr['width'], height=attr['height'],
                                      focal=attr['focal'])
                return cam
            elif tag.lower() == 'rgbd':
                rgbd = model.newRGBD (id=attr['id'], width=attr['width'], height=attr['height'],
                                      focal=attr['focal'], noise=attr['noise'], port=attr['pose'],
                                      ifconfig=attr['ifconfig'], parent=node)
                return rgbd
            elif tag.lower() == 'imu':
                imu = model.newIMU (id=attr['id'], port=attr['port'], parent=node)
                return imu
            elif tag.lower() == 'laser':
                laser = model.newLaser (id=attr['id'], port=attr['port'], min=attr['min'],
                                        max=attr['max'], angle=attr['angle'], measures=attr['measures'],
                                        ifconfig=attr['ifconfig'], parent=node)
                return laser
            elif tag.lower() == 'mesh':
                mesh = model.newMesh (id=attr['id'], path=attr['path'], scalex=attr['scalex'],
                                      scaley=attr['scaley'], scalez=attr['scalez'], render=attr['render'],
                                      tx=attr['tx'], ty=attr['ty'], tz=attr['tz'], rx=attr['rx'],
                                      ry=attr['ry'], rz=attr['rz'], collidable=attr['collidable'],
                                      parent=node)
                return mesh
            elif tag.lower() == 'pointcloud':
                pc = model.newPointCloud (id=attr['id'], parent=node)
                return pc
            elif tag.lower() == 'innermodel':
                raise Exception ("Tag <innermodel> can only be the root tag.")
            elif tag.lower() == 'plane':
                plane = model.newPlane (id=attr['id'], texture=attr['texture'], width=attr['width'],
                                    height=attr['height'], depth=attr['depth'], repeat=attr['repeat'],
                                    nx=attr['nx'], ny=attr['ny'], nz=attr['nz'], px=attr['px'],
                                    py=attr['py'], pz=attr['pz'], collidable=attr['collidable'],
                                    parent=node)
                return plane
            elif tag.lower() == 'include':
                InnerModelReader.include (attr['path'], model, node)
            elif tag.lower() == 'axes':
                defaultLength = attr['length']
                defaultWidth = attr['width']

                lengths = []
                widths = []

                for _ in range (3):
                    lengths.append (200 if defaultLength < 0 else defaultLength)
                    widths.append (15 if defaultWidth < 0 else defaultWidth)

                xLength = attr['xlength']
                if (xLength > 0):
                    lengths[0] = xLength

                xWidth = attr['xwidth']
                if (xWidth > 0):
                    widths[0] = xWidth

                yLength = attr['xlength']
                if (yLength > 0):
                    lengths[1] = yLength

                yWidth = attr['xwidth']
                if (yWidth > 0):
                    widths[1] = yWidth

                zLength = attr['xlength']
                if (zLength > 0):
                    lengths[2] = zLength

                zWidth = attr['xwidth']
                if (zWidth > 0):
                    widths[2] = zWidth

                plane = model.newPlane (id=attr['id']+'x', parent=node, texture="#ff0000",
                                        width=widths[0], height=widths[0], depth=lengths[0],
                                        repeat=1, nx=1, ny=0, nz= 0, px=lengths[0]/2, py=0, pz=0,
                                        collidable=False)
                node.addChild (plane)
                node.innerModel = plane.model = model

                plane = model.newPlane (id=attr['id']+'y', parent=node, texture="#00ff00",
                                        width=widths[1], height=lengths[1], depth=widths[1],
                                        repeat=1, nx=1, ny=0, nz= 0, px=0, py=lengths[1]/2, pz=0,
                                        collidable=False)
                node.addChild (plane)
                node.innerModel = plane.model = model

                plane = model.newPlane (id=attr['id']+'z', parent=node, texture="#0000ff",
                                        width=lengths[2], height=widths[2], depth=widths[2],
                                        repeat=1, nx=1, ny=0, nz= 0, px=0, py=0, pz=lengths[2]/2,
                                        collidable=False)
                node.addChild (plane)
                node.innerModel = plane.model = model

                plane = model.newPlane (id=attr['id']+'c', parent=node, texture="#ffffff",
                                        width=widths[0]*1.3, height=widths[1]*1.3, depth=widths[2]*1.3,
                                        repeat=1, nx=1, ny=0, nz= 0, px=0, py=0, pz=0, collidable=False)
                node.addChild (plane)
                node.innerModel = plane.model = model

                return plane
            elif tag.lower() == 'display':
                display = model.newDisplay (id=attr['id'], port=attr['port'], texture=attr['texture'],
                                    width=attr['width'], height=attr['height'], depth=attr['depth'],
                                    repeat=attr['repeat'], nx=attr['nx'], ny=attr['ny'], nz=attr['nz'],
                                px=attr['px'], py=attr['py'], pz=attr['pz'], collidable=attr['collidable'],
                                parent=node)
                return display
            else:
                print ("%s is not a valid tag name", tag)
                return None

    @staticmethod
    def recursive (parentDomNode: 'ET', model: 'InnerModel', imNode: 'InnerModelNode'):
        for child in parentDomNode:
            node = InnerModelReader.getClass (model, imNode, child.tag, child.attrib)
            imNode.addChild (node)
            node.innerModel = model
            InnerModelReader.recursive (child, model, node)

    def getValidNodeAttributes (self):
        return self.validAttr

