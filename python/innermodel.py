from innermodelnode import InnerModelNode
from innermodelmatrix import InnerModelMatrix
from innermodeltransform import InnerModelTransform
from innermodelmesh import InnerModelMesh
from innermodelrtmatrix import InnerModelRTMat
from innermodelvector import InnerModelVector

class InnerModel(object):
    def __init__ (self, xmlFilePath: str = None, im: 'InnerModel' = None):
        self.path = xmlFilePath
        self.root = None
        self.hash = None
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
        pass

    def newMesh (self, id: str, parent: 'InnerModelNode', path: str,
                scale: float, render: int, tx: float, ty: float, tz: float,
                rx: float, ry: float, rz: float, collidable: bool) -> 'InnerModelMesh':
        pass

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
