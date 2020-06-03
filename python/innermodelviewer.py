'''
For Rendering the innermodel scene
Author: ksakash@github.com (Akash Kumar Singh)
'''

import math
import time
import numpy as np
import pybullet as p
import pybullet_data

from innermodel import InnerModel
from innermodelnode import InnerModelNode
from innermodelvector import InnerModelVector
from innermodelmesh import InnerModelMesh
from innermodelplane import InnerModelPlane
from innermodeldifferentialrobot import InnerModelDifferentialRobot
from innermodeltransform import InnerModelTransform
from innermodeljoint import InnerModelJoint
from innermodelomnirobot import InnerModelOmniRobot
from innermodelprismaticjoint import InnerModelPrismaticJoint

class MultiLinkedBody (object):
    '''Class to represent a Multi Linked Body'''

    def __init__(self):
        '''Constructor having all the paramters for multibody in pybullet'''

        self.id = ''
        self.baseMass = 0
        self.baseCollisionShapeIndex = -1
        self.baseVisualShapeIndex = -1
        self.basePosition = [0,0,0]
        self.baseOrientation = p.getQuaternionFromEuler([0,0,0])
        self.linkMasses = []
        self.linkCollisionShapeIndices = []
        self.linkVisualShapeIndices = []
        self.linkPositions = []
        self.linkOrientations = []
        self.linkInertialFramePositions = []
        self.linkInertialFrameOrientations = []
        self.linkParentIndices = []
        self.linkJointTypes = []
        self.linkJointAxis = []

    def eulerFromNormal (self, normal):
        '''Returns rotation angles from normal vector of a plane'''
        normal = [normal[0], normal[1], normal[2]]
        if normal == [0, 0, 1]:
            euler = [0, 0, 0]
        elif normal == [0, 0, -1]:
            euler = [np.pi, 0, 0]
        elif normal == [1, 0, 0]:
            euler = [-np.pi/2, 0, -np.pi/2]
        elif normal == [-1, 0, 0]:
            euler = [-np.pi/2, 0, np.pi/2]
        elif normal == [0, 1, 0]:
            euler = [-np.pi/2, 0, 0]
        elif normal == [0, -1, 0]:
            euler = [np.pi/2, 0, 0]
        else:
            euler = [0, 0, 0]
        return euler

    def hexToInt (self, c):
        if (c == '1' or c == '2' or c == '3' or c == '4' or c == '5' \
            or c == '6' or c == '7' or c == '8' or c == '9' or c == '0'):
            return int (c)
        elif (c == 'A' or c == 'a'):
            return 10
        elif (c == 'B' or c == 'b'):
            return 11
        elif (c == 'C' or c == 'c'):
            return 12
        elif (c == 'D' or c == 'd'):
            return 13
        elif (c == 'E' or c == 'e'):
            return 14
        elif (c == 'F' or c == 'f'):
            return 15
        else:
            print (c)
            raise Exception ("invalid character")

    def readTexture (self, texture):
        '''Convert the texture into the color format taken by pybullet'''

        r = float (self.hexToInt(texture[1])*16 + self.hexToInt(texture[2]))/255.0
        g = float (self.hexToInt(texture[3])*16 + self.hexToInt(texture[4]))/255.0
        b = float (self.hexToInt(texture[5])*16 + self.hexToInt(texture[5]))/255.0
        color = [r, g, b, 1]
        return color

    def includeBody (self, mass, visual, collision, position, orientation, parentId, joint):
        '''Include a body with given parameters to the list'''

        self.linkMasses.append (mass)
        self.linkCollisionShapeIndices.append (collision)
        self.linkVisualShapeIndices.append (visual)
        self.linkPositions.append (position)
        self.linkOrientations.append (orientation)
        self.linkInertialFramePositions.append ([0,0,0])
        self.linkInertialFrameOrientations.append ([0,0,0,1])
        self.linkParentIndices.append (parentId)
        self.linkJointTypes.append (joint)
        self.linkJointAxis.append ([0,0,1])

    def recursiveConstructor (self, node, tr, parentId):
        '''Recursively construct all the parts in a body'''

        if isinstance (node, InnerModelMesh):
            linkMass = 1
            self.baseMass += linkMass
            meshScale = [node.scalex, node.scaley, node.scalez]
            visualShapeId = p.createVisualShape (shapeType=p.GEOM_MESH,
                                                 fileName=node.meshPath,
                                                 meshScale=meshScale)
            collisionShapeId = p.createCollisionShape (shapeType=p.GEOM_MESH,
                                                       fileName=node.meshPath,
                                                       meshScale=meshScale)
            linkPosition = [node.tx/100.0 + tr.x(), node.ty/100.0 + tr.y(), node.tz/100.0 + tr.z()]
            linkOrientation = p.getQuaternionFromEuler ([node.rx + tr.rx(), node.ry + tr.ry(),
                                                         node.rz + tr.rz()])
            linkJointType = p.JOINT_FIXED
            self.includeBody (linkMass, visualShapeId, collisionShapeId, linkPosition,
                              linkOrientation, parentId, linkJointType)
        elif isinstance (node, InnerModelPlane):
            linkMass = 1
            self.baseMass += linkMass
            size = [node.width/2,node.height/2,node.depth/2]
            color = self.readTexture (node.texture)
            visualShapeId = p.createVisualShape (shapeType=p.GEOM_BOX,
                                                 rgbaColor=color,
                                                 halfExtents=size)
            collisionShapeId = p.createCollisionShape (shapeType=p.GEOM_BOX,
                                                       halfExtents=size)
            linkPosition = [node.point[0]/100.0 + tr.x(), node.point[1]/100.0 + tr.y(),
                            node.point[2]/100.0 + tr.z()]
            euler = self.eulerFromNormal (node.normal)
            linkOrientation = p.getQuaternionFromEuler (euler)
            linkJointType = p.JOINT_FIXED
            self.includeBody (linkMass, visualShapeId, collisionShapeId, linkPosition,
                              linkOrientation, parentId, linkJointType)
        else:
            self.baseMass += node.mass
            tr_ = InnerModelVector.vec6d (node.tx/100.0 + tr.x(), node.ty/100.0 + tr.y(),
                                          node.tz/100.0 + tr.z(), node.rx + tr.rx(),
                                          node.ry + tr.ry(), node.rz + tr.rz())
            for child in node.children:
                self.recursiveConstructor (child, tr_, parentId)

    def getClassType (self, node):
        '''Returns if an object is transform type or node type'''

        if isinstance (node, (InnerModelTransform, InnerModelDifferentialRobot, InnerModelJoint, \
                       InnerModelPrismaticJoint, InnerModelOmniRobot)):
            return 'Transform'
        else:
            return 'Node'

    def createBody (self, node):
        '''Create a separate body from the node'''

        assert (self.getClassType (node) == 'Transform')
        self.id = node.id
        # self.baseMass = node.mass
        self.baseMass = 10
        self.basePosition = [node.tx/100.0, node.ty/100.0, node.tz/100.0]
        self.baseOrientation = p.getQuaternionFromEuler ([node.rx, node.ry, node.rz])
        tr = InnerModelVector.vec6d (0, 0, 0, 0, 0, 0)
        parentId = 0

        for child in node.children:
            self.recursiveConstructor (child, tr, parentId)

class InnerModelViewer (object):
    '''Class to view the rendered world from innermodel'''

    def __init__ (self, innermodel: 'InnerModel' = None):
        '''Constructor, takes innermodel'''

        self.innerModel = innermodel
        self.bodies = []
        self.worldNode = None
        if self.innerModel is not None:
            self.construct (self.innerModel.root)

    def MakeMultiLinkedBody (self, node):
        '''Method to make a multilinked body'''

        body = MultiLinkedBody()
        body.createBody (node)
        return body

    def MakeWorld (self):
        '''To construct the world with all the bodies in it'''

        if self.worldNode is None:
            self.construct (self.innerModel.root)
        root = self.worldNode

        for child in root.children:
            body = self.MakeMultiLinkedBody (child)
            self.bodies.append (body)

    def construct (self, node) -> bool:
        '''Find the node representing the world'''

        if (node.id == 'world'):
            self.worldNode = node
            return True
        else:
            for child in node.children:
                if self.construct (child):
                    return True
            return False

    def render (self):
        '''Render the scene'''

        p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.loadURDF("plane.urdf")

        self.MakeWorld()
        p.setGravity(0, 0, -10)
        p.setRealTimeSimulation(1)
        print (len(self.bodies))

        for body in self.bodies:
            p.createMultiBody(baseMass=body.baseMass,
                            baseCollisionShapeIndex=body.baseCollisionShapeIndex,
                            baseVisualShapeIndex=body.baseVisualShapeIndex,
                            basePosition=body.basePosition,
                            baseOrientation=body.baseOrientation,
                            linkMasses=body.linkMasses,
                            linkCollisionShapeIndices=body.linkCollisionShapeIndices,
                            linkVisualShapeIndices=body.linkVisualShapeIndices,
                            linkPositions=body.linkPositions,
                            linkOrientations=body.linkOrientations,
                            linkInertialFramePositions=body.linkInertialFramePositions,
                            linkInertialFrameOrientations=body.linkInertialFrameOrientations,
                            linkParentIndices=body.linkParentIndices,
                            linkJointTypes=body.linkJointTypes,
                            linkJointAxis=body.linkJointAxis)

        while (1):
            p.stepSimulation()
            time.sleep(1. / 240.)

InnerModelViewer()
