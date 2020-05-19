from innermodelnode import InnerModelNode
from innermodelmatrix import InnerModelMatrix
from innermodeltransform import InnerModelTransform
from innermodelmesh import InnerModelMesh
from innermodelrtmatrix import InnerModelRTMat
from innermodelvector import InnerModelVector
from innermodeltouchsensor import InnerModelTouchSensor
from innermodeljoint import InnerModelJoint
from innermodelprismaticjoint import InnerModelPrismaticJoint
from innermodeldifferentialrobot import InnerModelDifferentialRobot
from innermodelomnirobot import InnerModelOmniRobot
from innermodelcamera import InnerModelCamera
from innermodelrgbd import InnerModelRGBD
from innermodelimu import InnerModelIMU
from innermodellaser import InnerModelLaser
from innermodelplane import InnerModelPlane
from innermodeldisplay import InnerModelDispay
from innermodelmesh import InnerModelMesh
from innermodelpointcloud import InnerModelPointCloud

class InnerModel(object):
    def __init__ (self, xmlFilePath: str = None, im: 'InnerModel' = None):
        self.path = xmlFilePath
        self.root = None
        self.hash = dict()
        self.localHashTr = None
        self.localHashRot = None
        self.listA = None
        self.listB = None

    def open (self, xmlFilePath: str) -> bool:
        pass

    def save (self, path: str) -> bool:
        pass

    def copy (self) -> 'InnerModel':
        pass

    def changeMesh (self, new_id: str, node: 'InnerModel'):
        pass

    def setRoot (self, node: 'InnerModelNode'):
        pass

    def update (self):
        pass

    def updateRotation (self, rotationId: str, rx: float, ry: float, rz: float):
        pass

    def updateTranslation (self, translationId: str, tx: float, ty: float, tz: float):
        pass

    def updatePlane (self, planeId: str, nx: float, ny: float, nz: float,
                                         px: float, py: float, pz: float):
        pass

    def updateTransform (self, transformId: str, tx: float, ty: float, tz: float,
                                                 rx: float, ry: float, rz: float):
        pass

    def cleanUpTables (self):
        pass

    def updateJointValue (self, jointId: str, angle: float, force: bool = False):
        pass

    def updatePrismaticJointPosition (self, jointId: str, position: float):
        pass

    def updateDisplay (self, displayId: str, texture: str):
        pass

    def newTransform (self, id: str, engine: str, parent: 'InnerModelNode',
                    mass: float, tx: float, ty: float, tz: float, rx: float,
                    ry: float, rz: float) -> 'InnerModelTransform':
        if (id in self.hash):
            raise Exception ("InnerModel.newTransform: Error: Trying to insert a node with an \
                              already-existing key: %s", id)

        newnode = InnerModelTransform (id=id, engine=engine, tx=tx, ty=ty, tz=tz,
                                        rx=rx, ry=ry, rz=rz, mass=mass, parent=parent)
        self.hash[id] = newnode
        return newnode

    def newTouchSensor (self, id: str, parent: 'InnerModelTransform', stype: str,
                        nx: float, ny: float, nz: float, min: float, max: float, port: int):
        if (id in self.hash):
            raise Exception ("InnerModel.newTouchSensor: Error: Trying to insert a node with an \
                              already-existing key: %s", id)
        newnode = InnerModelTouchSensor (id=id, stype=stype, nx=nx, ny=ny, nz=nz, min=min, max=max,
                                        port=port, parent=parent)
        self.hash[id] = newnode
        return newnode

    def newJoint (self, id: str, parent: 'InnerModelTransform', lx: float, ly: float, lz: float,
                  hx:float, hy: float, hz: float, tx: float, ty: float, tz: float, rx: float, ry: float,
                  rz: float, min: float, max: float, port: int, axis: str, home: float):
        if (id in self.hash):
            raise Exception ("InnerModel.newJoint: Error: Trying to insert a node with an \
                              already-existing key: %s", id)
        newnode = InnerModelJoint (id=id, lx=lx, ly=ly, lz=lx, hx=hx, hy=hy, hz=hz, tx=tx, ty=ty,
                                   tz=tz, rx=rx, ry=ry, rz=rz, min=min, max=max, port=port, axis=axis,
                                   home=home, parent=parent)
        self.hash[id] = newnode
        return newnode

    def newPrismaticJoint (self, id, min, max, value, offset, port, axis, home, parent):
        if (id in self.hash):
            raise Exception ("InnerModel.newPrismaticJoint: Error: Trying to insert a node with an \
                              already-existing key: %s", id)
        newnode = InnerModelPrismaticJoint (id, min, max, value, offset, port, axis, home, parent)
        self.hash[id] = newnode
        return newnode

    def newDifferentialRobot (self, id, tx, ty, tz, rx, ry, rz, port, noise, collide, parent):
        if (id in self.hash):
            raise Exception ("InnerModel.newDifferentialRobot: Error: Trying to insert a node with \
                              an already-existing key: %s", id)
        newnode = InnerModelDifferentialRobot (id, tx, ty, tz, rx, ry, rz, port, noise, collide, parent)
        self.hash[id] = newnode
        return newnode

    def newOmniRobot (self, id, tx, ty, tz, rx, ry, rz, port, noise, collide, parent):
        if (id in self.hash):
            raise Exception ("InnerModel.newOmniRobot: Error: Trying to insert a node with an \
                              already-existing key: %s", id)
        newnode = InnerModelOmniRobot (id, tx, ty, tz, rx, ry, rz, port, noise, collide, parent)
        self.hash[id] = newnode
        return newnode

    def newCamera (self, id, width, height, focal):
        if (id in self.hash):
            raise Exception ("InnerModel.newCamera: Error: Trying to insert a node with an \
                              already-existing key: %s", id)
        newnode = InnerModelCamera (id, width, height, focal, self, parent=None)
        self.hash[id] = newnode
        return newnode

    def newRGBD (self, id, width, height, focal, noise, port, ifconfig, parent):
        if (noise < 0):
            raise Exception ("InnerModel.newRGBD: noise can't have negative values")
        if (id in self.hash):
            raise Exception ("InnerModel.newRGBD: Error: Trying to insert a node with an \
                              already-existing key: %s", id)
        newnode = InnerModelRGBD (id, width, height, focal, noise, port, ifconfig, self, parent)
        self.hash[id] = newnode
        return newnode

    def newIMU (self, id, port, parent):
        if (id in self.hash):
            raise Exception ("InnerModel.newIMU: Error: Trying to insert a node with an \
                              already-existing key: %s", id)
        newnode = InnerModelIMU (id, port, parent)
        self.hash[id] = newnode
        return newnode

    def newLaser (self, id, port, min, max, angle, measures, ifconfig, parent):
        if (id in self.hash):
            raise Exception ("InnerModel.newLaser: Error: Trying to insert a node with an \
                              already-existing key: %s", id)
        newnode = InnerModelLaser (id, port, min, max, angle, measures, ifconfig, self, parent)
        self.hash[id] = newnode
        return newnode

    def newMesh (self, id: str, parent: 'InnerModelNode', path: str,
                scalex: float, scaley: float, scalez: float, render: int, tx: float, ty: float,
                tz: float, rx: float, ry: float, rz: float, collidable: bool) -> 'InnerModelMesh':
        if (id in self.hash):
            raise Exception ("InnerModel.newMesh: Error: Trying to insert a node with an \
                              already-existing key: %s", id)
        newnode = InnerModelMesh (id, path, scalex, scaley, scalez, render, tx, ty, tz, rx, ry, rz,
                                  collidable, parent)
        self.hash[id] = newnode
        return newnode

    def newPointCloud (self, id, parent):
        if (id in self.hash):
            raise Exception ("InnerModel.newPointCloud: Error: Trying to insert a node with an \
                              already-existing key: %s", id)
        newnode = InnerModelPointCloud (id, parent)
        self.hash[id] = newnode
        return newnode

    def newPlane (self, id, texture, width, height, depth, repeat, nx, ny, nz, px, py, pz, collidable,
                  parent):
        if (id in self.hash):
            raise Exception ("InnerModel.newPlane: Error: Trying to insert a node with an \
                              already-existing key: %s", id)
        newnode = InnerModelPlane (id, texture, width, height, depth, repeat, nx, ny, nz, px, py, pz,
                                   collidable, parent)
        self.hash[id] = newnode
        return newnode

    def newDisplay (self, id, port, texture, width, height, depth, repeat, nx, ny, nz, px, py, pz,
                    collidable, parent):
        if (id in self.hash):
            raise Exception ("InnerModel.newDisplay: Error: Trying to insert a node with an \
                              already-existing key: %s", id)
        newnode = InnerModelDispay (id, port, texture, width, height, depth, repeat, nx, ny, nz,
                                    px, py, pz, collidable, parent)
        self.hash[id] = newnode
        return newnode

    def getTransform (self, id: str) -> 'InnerModelTransform':
        pass

    def getMesh (self, id: str) -> 'InnerModelMesh':
        pass

    def transform (self, destId: str, origId: str,
                origVec: 'InnerModelVector' = None) -> 'InnerModelVector':
        pass

    def transform6D (self, destId: str, orgId: str,
                    origVec: 'InnerModelVector' = None) -> 'InnerModelVector':
        pass

    def getTransformationMatrix (self, destId: str, origId: str) -> 'InnerModelRTMat':
        pass

    def getRotationMatrixTo (self, to: str, fr: str):
        pass

    def getTranslationMatrixTo (self, to: str, fr: str):
        pass

    def rotationAngles (self, destId: str, origId: str) -> 'InnerModelVector':
        pass

    def getIDKeys (self):
        pass

    def getNode (self):
        pass

    def removeSubTree (self, item, list):
        pass

    def removeNode (self, id):
        pass

    def moveSubTree (self, nodeSrc, nodeDst):
        pass

    def getSubTree (self, node, list):
        pass

    def computeLevels (self, node):
        pass

    def getRoot (self):
        pass

    def getParentIdentifier (self):
        pass

    def printTree (self, s):
        pass

    def setLists (self, origId, destId):
        pass
