'''
Created on Sep 13, 2019

@author: marco
'''
import numpy as np
import sympy
import logging, time, math

import meshcat
import meshcat.geometry as meshcatg

import kgprim.ct.frommotions as frommotions
import kgprim.ct.repr.mxrepr as mxrepr

import robmodel.jposes
import robmodel.treeutils as treeu


logger = logging.getLogger(__name__)

class CustomCommand:
    def __init__(self, path):
        self.path = path
    def lower(self):
        return {
            u"type": u"custom",
            u"path": self.path.lower(),
            u"data": u"aaa"
        }

class MeshCatScene:
    def __init__(self, robotGeometry, tree, robotTransforms, meshcatVisualizer):
        self.vis = meshcatVisualizer
        self.rob = tree.robot
        self.tree= tree
        self.geom= robotGeometry
        self.tf  = robotTransforms

        self.jointTransforms = {}
        self.linkByJoint = {}

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
                jframe    = mcatJoint["jframe"]
                jframe.window.send( CustomCommand(jframe.path) ) # slight hack, since there is not really a proper message in meshcat for frames

                # Geometrical data, ie the fixed pose of the joint frame
                # relative to the predecessor link frame
                pose  = self.geom.byJoint[ joint ]  # joint_wrt_predecessor
                parent_CT_joint = frommotions.toCoordinateTransform(pose)
                parent_H_joint  = mxrepr.hCoordinatesNumeric(parent_CT_joint)
                mcatJoint.set_transform( parent_H_joint )


                tr = self.tf.jointTransform(joint)
                tr = mxrepr.hCoordinatesSymbolic(tr)
                self.jointTransforms[joint.name] = tr;
                if len(tr.variables) > 0 :
                    aux = tr.eval(0.0)
                    mcatLink.set_transform( aux[:,:] )
                # If there are no variables, it means this is the joint
                # transform of a bloody fixed joint, so we do not have to do anything

                self.linkByJoint[joint.name] = mcatLink

            if link.name in meshesPath :
                mesh = mcatLink["mesh"]
                mesh.set_object(meshcatg.StlMeshGeometry.from_file( meshesPath[link.name] ))

                link_H_stl = link_H_mesh.get(link.name, np.identity(4))
                mesh.set_transform( link_H_stl )
            else :
                logger.warning("Could not find mesh for link '{0}'".format(link.name))

            frame = mcatLink["lframe"]
            frame.window.send( CustomCommand(frame.path) ) # slight hack, since there is not really a proper message in meshcat for frames


            for child in self.tree.children( link ) :
                addToScene(child, link, mcatLink)

        # start the recursive calls:
        addToScene(self.rob.base, None, self.vis)

    def setJointStatus(self, q):
        for i in range(0, len(q)) :
            joint = self.tree.supportingJoint( self.rob.codeToLink[i+1] )
            tr = self.jointTransforms[joint.name];
            if len(tr.variables) > 0 :
                numval = tr.eval( q[i] )
                mcatLink = self.linkByJoint[joint.name]
                mcatLink.set_transform( numval )


def trivialJointMotion(bridgeToScene):
    t = 0
    #q = sympy.Symbol('q')
    uin = '.'
    while uin != 'q':
        time.sleep(0.2)
#        uin = input('enter q to terminate, r to reload: ')
        t = t + 0.2
        q = 0.5 * math.sin(t)
        bridgeToScene.setJointStatus( [q,q,q] )


def start(robotGeometryModel, meshesPaths, meshesPoses):
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
    tr   = robmodel.jposes.JointPoses(robot, robotGeometryModel.framesModel)

    def init():
        meshcatv = meshcat.Visualizer()
        loader   = MeshCatScene(robotGeometryModel, tree, tr, meshcatv)
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
