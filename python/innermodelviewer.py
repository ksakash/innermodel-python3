'''
For Rendering the innermodel scene
Author: ksakash@github.com (Akash Kumar Singh)
'''

import math
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

        euler = [0,0,0]
        return euler

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
        self.linkJointAxis.append ([0,0,0])

    def recursiveConstructor (self, node, tr, parentId):
        '''Recursively construct all the parts in a body'''

        if isinstance (node, InnerModelMesh):
            linkMass = node.mass
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
            linkMass = node.mass
            size = [node.width/2,node.height/2,node.depth/2]
            visualShapeId = p.createVisualShape (shapeType=p.GEOM_BOX,
                                                 halfExtents=size)
            collisionShapeId = p.createCollisionShape (shapeType=p.GEOM_BOX,
                                                       halfExtents=size)
            linkPosition = [node.px + tr.x(), node.py + tr.y(), node.pz + tr.z()]
            euler = self.eulerFromNormal (node.normal)
            linkOrientation = p.getQuaternionFromEuler (euler)
            linkJointType = p.JOINT_FIXED
            self.includeBody (linkMass, visualShapeId, collisionShapeId, linkPosition,
                              linkOrientation, parentId, linkJointType)
        else:
            tr_ = InnerModelVector.vec6d (node.tx + tr.x(), node.ty + tr.y(), node.tz + tr.z(),
                                          node.rx + tr.rx(), node.ry + tr.ry(), node.rz + tr.rz())
            self.recursiveConstructor (node, tr_, parentId)

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

        for child in node.chilren:
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

    def getRayFromTo(self, mouseX, mouseY):
        width, height, _, _, _, camForward, horizon, vertical, _, _, dist, \
                                                            camTarget = p.getDebugVisualizerCamera()
        camPos = [
            camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
            camTarget[2] - dist * camForward[2]
        ]
        farPlane = 10000
        rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
        invLen = farPlane * 1. / (math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] *
                                            rayForward[1] + rayForward[2] * rayForward[2]))
        rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
        rayFrom = camPos
        oneOverWidth = float(1) / float(width)
        oneOverHeight = float(1) / float(height)
        dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
        dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
        rayToCenter = [
            rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
        ]
        rayTo = [
            rayToCenter[0] - 0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] -
            float(mouseY) * dVer[0], rayToCenter[1] - 0.5 * horizon[1] + 0.5 * vertical[1] +
            float(mouseX) * dHor[1] - float(mouseY) * dVer[1], rayToCenter[2] - 0.5 * horizon[2] +
            0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
        ]
        return rayFrom, rayTo

    def render (self):
        '''Render the scene'''

        cid = p.connect(p.SHARED_MEMORY)
        if (cid < 0):
            p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(numSolverIterations=10)
        p.setTimeStep(1. / 120.)
        logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")
        #useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)
        p.loadURDF("plane100.urdf", useMaximalCoordinates=True)
        #disable rendering during creation.
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        #disable tinyrenderer, software (CPU) renderer, we don't use it here
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

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
                            linkJointAxis=body.axis)

        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.stopStateLogging(logId)
        p.setGravity(0, 0, -10)
        p.setRealTimeSimulation(1)

        colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 1, 1]]
        currentColor = 0

        while (1):
            mouseEvents = p.getMouseEvents()
            for e in mouseEvents:
                if ((e[0] == 2) and (e[3] == 0) and (e[4] & p.KEY_WAS_TRIGGERED)):
                    mouseX = e[1]
                    mouseY = e[2]
                    rayFrom, rayTo = self.getRayFromTo(mouseX, mouseY)
                    rayInfo = p.rayTest(rayFrom, rayTo)

                    for l in range(len(rayInfo)):
                        hit = rayInfo[l]
                        objectUid = hit[0]
                        if (objectUid >= 1):
                            p.changeVisualShape(objectUid, -1, rgbaColor=colors[currentColor])
                            currentColor += 1
                            if (currentColor >= len(colors)):
                                currentColor = 0
