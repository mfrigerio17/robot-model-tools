'''
Created on Sep 13, 2019

@author: marco
'''
import numpy as np
import sympy
import logging, time, math, itertools

import meshcat
import meshcat.geometry as meshcatg

from kgprim.core import Pose
import kgprim.ct.frommotions as frommotions
import kgprim.ct.repr.mxrepr as mxrepr
import kgprim.ct.metadata

import robmodel.jposes
import robmodel.treeutils as treeu


logger = logging.getLogger(__name__)

class CustomCommand:
    def __init__(self, path, subcmd=""):
        self.path = path
        self.subcmd = subcmd
    def lower(self):
        return {
            u"type"   : u"plugin",
            u"pluginname" : u"RobModelExtras",
            u"path": self.path.lower(),
            u"data": u"aaa",
        u"subtype" : self.subcmd.lower()
        }

class MeshCatScene:
    def __init__(self, robotGeometry, tree, robotTransforms, meshcatVisualizer, robotGeometryParams=None):
        self.vis = meshcatVisualizer
        self.rob = tree.robot
        self.tree= tree
        self.geom= robotGeometry
        self.tf  = robotTransforms
        self.robotGeometryParams = robotGeometryParams

        self.jointTransforms = {}
        self.linkByJoint = {}
        self.linkByName  = {}

    def load(self, meshesPath, link_H_mesh={}):
        '''
        Load the object meshes and add them to the scene.

        Keyword arguments:
        `meshesPath`: a dictionary indexed by the robot links names, whose
            values must be valid filesystem paths for the mesh files.
        `link_H_meshes`: another dictionary with the same keys, whose values are
                         the homogeneous coordinate transforms mapping mesh
                         coordinates to robot-model-link coordinates
        '''

        def addToScene(link, parent, meshCatParent):
            if parent is None : # the base link of the robot; the meshcat parent is thus the world
                mcatLink = meshCatParent[ link.name ]
            else :
                # The local visualization tree that we have to construct:
                # parent \
                #        |- joint \
                #                 |- joint frame
                #                 |- link \
                #                         |- link frame
                #                         |- mesh
                joint     = self.rob.linkPairToJoint(link, parent)
                mcatJoint = meshCatParent[ joint.name ]
                mcatLink  = mcatJoint[ link.name ]
                jframe    = mcatJoint[joint.name + "_frame"]
                jframe.window.send( CustomCommand(jframe.path, "frame") )

                # Geometrical data, ie the fixed pose of the joint frame
                # relative to the predecessor link frame
                pose  = self.geom.byJoint[ joint ]  # joint_wrt_predecessor
                parent_CT_joint = frommotions.toCoordinateTransform(pose)
                mcatJoint.set_transform( self._getNumericalMatrix(parent_CT_joint) )

                tr = self.tf.jointTransform(joint)
                tr = mxrepr.hCoordinatesSymbolic(tr)
                self.jointTransforms[joint.name] = tr;
                if len(tr.variables) > 0 :
                    aux = tr.eval(0.0)
                    mcatLink.set_transform( aux[:,:] )
                # If there are no variables, it means this is the joint
                # transform of a bloody fixed joint, so we do not have to do anything

                self.linkByJoint[joint.name] = mcatLink
            self.linkByName[link.name] = mcatLink

            if link.name in meshesPath :
                mesh = mcatLink["mesh"]
                mesh.set_object(meshcatg.StlMeshGeometry.from_file( meshesPath[link.name] ))

                link_H_stl = link_H_mesh.get(link.name, np.identity(4))
                mesh.set_transform( link_H_stl )
                mcatLink.window.send( CustomCommand(mcatLink.path, "link-with-mesh") )
            else :
                logger.warning("Could not find mesh for link '{0}'".format(link.name))

            frame = mcatLink[link.name + "_frame"]
            frame.window.send( CustomCommand(frame.path, "frame") )

            for child in self.tree.children( link ) :
                addToScene(child, link, mcatLink)

        # start the recursive calls to add the links:
        addToScene(self.rob.base, None, self.vis)

        # add the additional user frames attached to the links
        frames = self.geom.framesModel
        for uFrame in frames.userAttachedFrames :
            userFrame = frames.framesByName[ uFrame.name ]
            linkFrame = frames.framesByName[ uFrame.body.name ]
            pose = Pose(target=userFrame, reference=linkFrame)
            if pose in self.geom.byPose :
                poseSpec = self.geom.byPose[ pose ]
                mcatLink = self.linkByName[uFrame.body.name]
                # the frame is a new child of the link, in the scene tree
                frame = mcatLink[userFrame.name]
                # send the command to visualize a Cartesian frame
                cmd = CustomCommand(frame.path, "frame")
                frame.window.send( cmd )
                # set the transform, to orient the frame relative to the link
                link_CT_frame = frommotions.toCoordinateTransform(poseSpec)
                frame.set_transform( self._getNumericalMatrix(link_CT_frame) )
            else :
                logger.warning("Could not find the motion model for pose '{0}'".format(pose))


    def setJointStatus(self, q):
        for i in range(0, len(q)) :
            joint = self.tree.supportingJoint( self.rob.codeToLink[i+1] )
            tr = self.jointTransforms[joint.name];
            if len(tr.variables) > 0 :
                numval = tr.eval( q[i] )
                mcatLink = self.linkByJoint[joint.name]
                mcatLink.set_transform( numval )

    def _getNumericalMatrix(self, ctransform):
        matrix = None
        ct_info = kgprim.ct.metadata.TransformMetadata(ctransform)
        if ct_info.is_parametric :
            if self.robotGeometryParams is None :
                raise RuntimeError("Cannot resolve parametric transforms without parameter values")

            matrix_with_symbols = mxrepr.hCoordinatesSymbolic(ctransform)
            pvalues = {}
            for p in ct_info.parameters: # this is a ordered set
                pvalues[p] = self.robotGeometryParams[p.name]
            matrix_with_symbols.setParametersValue(pvalues)
            matrix = matrix_with_symbols.eval().astype(float)
        else:
            matrix = mxrepr.hCoordinatesNumeric(ctransform).astype(float)
        return matrix



def trivialJointMotion(bridgeToScene):
    t = 0
    length = len(bridgeToScene.rob.joints)
    uin = '.'
    while uin != 'q':
        time.sleep(0.2)
#        uin = input('enter q to terminate, r to reload: ')
        t = t + 0.2
        q = 0.5 * math.sin(t)
        qstate = list(itertools.repeat(q, length))
        bridgeToScene.setJointStatus( qstate )


def start(robotGeometryModel, meshesPaths, meshesPoses, robotGeometryParams=None):
    robot = robotGeometryModel.connectivityModel
    frames= robotGeometryModel.framesModel
    meshes_H = {}
    if meshesPoses is not None:
        for poseSpec in meshesPoses.poses :
            skip = False
            linkFrame = poseSpec.pose.target    #
            meshFrame = poseSpec.pose.reference # initial assumption
            if linkFrame.name not in frames.framesByName.keys() :
                linkFrame = poseSpec.pose.reference
                meshFrame = poseSpec.pose.target
                if linkFrame.name not in frames.framesByName.keys() :
                    logger.warning("No frame of the mesh-pose '{0}' seems to be a robot frame".format(poseSpec.pose))
                    skip = True
            if not skip :
                # Explicitly ask for the transform from mesh coordinates to link coordinates
                link_CT_stl = frommotions.toCoordinateTransform(poseSpec, right_frame=meshFrame)
                meshes_H[ linkFrame.name ] = mxrepr.hCoordinatesNumeric(link_CT_stl)

    tree = treeu.TreeUtils(robot)
    tr   = robmodel.jposes.JointPoses(robot, frames, robotGeometryModel.jointAxes)

    def init():
        meshcatv = meshcat.Visualizer()
        loader   = MeshCatScene(robotGeometryModel, tree, tr, meshcatv, robotGeometryParams)
        meshcatv.wait();
        loader.load(meshesPaths, meshes_H)
        return meshcatv, loader
    meshcatv, loader = init();

    uin = '.'
    while uin != 'q':
        uin = input('enter q to terminate, r to reload: ')
        if(uin == 'r') :
            meshcatv.window.server_proc.kill()
            meshcatv.window.server_proc.wait()
            del meshcatv, loader
            meshcatv, loader = init()
        elif(uin == 'j') :
            trivialJointMotion(loader)
