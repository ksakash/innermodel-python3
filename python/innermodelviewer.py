'''
For Rendering the innermodel scene
Author: ksakash@github.com (Akash Kumar Singh)
'''

import math
import time
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

    # TODO
    def eulerFromNormal (self, normal):
        '''Returns rotation angles from normal vector of a plane'''

        euler = [0,0,0]
        return euler

    # TODO
    def readTexture (self, texture):
        color = [0, 0, 0, 1]
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
            linkPosition = [node.tx + tr.x(), node.ty + tr.y(), node.tz + tr.z()]
            linkOrientation = p.getQuaternionFromEuler ([node.rx + tr.rx(), node.ry + tr.ry(),
                                                         node.rz + tr.rz()])
            linkJointType = p.JOINT_FIXED
            self.includeBody (linkMass, visualShapeId, collisionShapeId, linkPosition,
                              linkOrientation, parentId, linkJointType)
        elif isinstance (node, InnerModelPlane):
            linkMass = 1
            self.baseMass += linkMass
            size = [node.width/2,node.height/2,node.depth/2]
            print (size)
            visualShapeId = p.createVisualShape (shapeType=p.GEOM_BOX,
                                                 halfExtents=size)
            collisionShapeId = p.createCollisionShape (shapeType=p.GEOM_BOX,
                                                       halfExtents=size)
            linkPosition = [node.point[0] + tr.x(), node.point[1] + tr.y(), node.point[2] + tr.z()]
            euler = self.eulerFromNormal (node.normal)
            linkOrientation = p.getQuaternionFromEuler (euler)
            linkJointType = p.JOINT_FIXED
            self.includeBody (linkMass, visualShapeId, collisionShapeId, linkPosition,
                              linkOrientation, parentId, linkJointType)
        else:
            self.baseMass += node.mass
            tr_ = InnerModelVector.vec6d (node.tx + tr.x(), node.ty + tr.y(), node.tz + tr.z(),
                                          node.rx + tr.rx(), node.ry + tr.ry(), node.rz + tr.rz())
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
        self.baseMass = node.mass
        self.basePosition = [node.tx, node.ty, node.tz]
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
