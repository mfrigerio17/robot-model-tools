import xml.etree.ElementTree as ET
from collections import OrderedDict as ODict
import logging, sys

import numpy as np
import math

import kgprim.core as primitives
import kgprim.motions as motions
from kgprim.motions import MotionSequence
from kgprim.motions import PoseSpec
from kgprim.motions import MotionPath

import robmodel.connectivity
import robmodel.ordering
import robmodel.frames
import robmodel.geometry
from robmodel.connectivity import JointKind

logger = logging.getLogger(__name__)

'''
Simply reads the XML file and stores the links/joints data, no conversions
'''
class URDFWrapper :
    class Link:
        def __init__(self, name):
            self.name    = name
            self.inertia = None
            self.parent  = None
            self.supportingJoint = None
    class Joint:
        def __init__(self, name):
            self.name = name
            self.type  = None
            self.frame = None
            self.parent= None
            self.child = None
            #self.predec_H_joint = np.identity(4)

    iMomentsLabels = ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']

    def __init__(self, urdfInFile):
        root = ET.parse(urdfInFile)
        self.robotName = root.getroot().get('name')

        linkNodes  = root.findall("link")
        jointNodes = root.findall("joint")

        self.links  = ODict()
        self.joints = ODict()
        self.frames = ODict()

        for nodelink in linkNodes:
            name = nodelink.get('name')
            link = URDFWrapper.Link( name )
            link.inertia = self.readInertialData(nodelink)
            self.links[name] = link

        for nodejoint in jointNodes:
            name = nodejoint.get('name')
            joint = URDFWrapper.Joint( name )
            joint.type  = nodejoint.get('type')
            joint.frame = self.readJointFrameData( nodejoint )
            #joint.predec_H_joint[:3,:3] = getR_extrinsicXYZ( * joint.frame['rpy'] )
            #joint.predec_H_joint[:3,3]  = np.array( joint.frame['xyz'] )
            joint.parent= nodejoint.find('parent').get('link')
            joint.child = nodejoint.find('child').get('link')

            # Note I keep URDF nomenclature ("parent" and "child") just to
            # stress the bond with the source URDF XML file. I will later use
            # the more appropriate terms (e.g. "predecessor")

            self.joints[name] = joint

            predecessor = self.links[ joint.parent ]
            successor   = self.links[ joint.child ]
            successor.parent = predecessor # a Link instance, not a name
            successor.supportingJoint = joint

    def readInertialData(self, linkNode):
        params = dict()
        paramsNode = linkNode.find('inertial')

        # Default inertia parameters if the URDF does not have the data
        if paramsNode == None :
            params['mass'] = 0.0
            params['xyz']  = (0.0, 0.0, 0.0)
            for m in URDFWrapper.iMomentsLabels :
                params[m] = 0.0
            return params

        mass = float(paramsNode.find('mass').get('value'))

        xyz = (0.0, 0.0, 0.0)
        originNode = paramsNode.find('origin')
        if originNode != None :
            comstr = originNode.get('xyz')
            if(comstr != None) :
                xyz = tuple([float(x) for x in comstr.split()])

            # We cannot deal with non-zero values for the 'rpy' attribute
            rpystr = originNode.get('rpy')
            if(rpystr != None) :
                tmp = [float(x) for x in rpystr.split()]
                if(sum(tmp) != 0) :
                    logger.warning('The rpy attribute in the inertial section is not yet supported (link ' + linkNode.get('name') + '). Ignoring it.')

        moments = paramsNode.find('inertia')
        for m in URDFWrapper.iMomentsLabels :
            params[m] = float(moments.get(m))

        params['mass'] = mass
        params['xyz']  = xyz
        return params


    def readJointFrameData(self, jointNode):
        params = dict()

        # URDF defaults:
        params['xyz'] = (0.,0.,0.)
        params['rpy'] = (0.,0.,0.)

        frameNode = jointNode.find('origin')
        if frameNode != None :
            xyz_node = frameNode.get('xyz')
            if xyz_node != None :
                params['xyz'] = tuple([float(x) for x in xyz_node.split()])
            rpy_node = frameNode.get('rpy')
            if rpy_node != None :
                params['rpy'] = tuple([float(x) for x in rpy_node.split()])

        axis_node = jointNode.find('axis')
        if axis_node != None :
            params['axis'] = tuple([float(x) for x in axis_node.get('xyz').split()])
        else :
            params['axis'] = (1.,0.,0.) # URDF default

        return params



def toValidID( name ) :
    return name.replace('-', '__')



def linkFrameToJointFrameInURDF(urdfjoint):
    '''
    Return the data about the location of the joint frame relative to the
    predecessor link frame.

    This function returns three values:
    - the `xyz` attribute as found in the source URDF
    - the `rpy` attribute as found in the source URDF
    - the rigid motion model representing the relative pose (this is the motion
      that the link frame should undergo to coincide with the joint frame)
    '''
    xyz = urdfjoint.frame['xyz']
    rpy = urdfjoint.frame['rpy']

    tr = [motions.translation(a, xyz[a.value]) for a in motions.Axis if round(xyz[a.value],5) != 0.0]
    rt = [motions.rotation   (a, rpy[a.value]) for a in motions.Axis if round(rpy[a.value],5) != 0.0]

    motion__linkToJoint = MotionSequence(tr+rt, MotionSequence.Mode.fixedFrame)
    return xyz, rpy, motion__linkToJoint

def convert( urdf ) :
    '''
    Reads the model from a URDFWrapper instance, and construct the corresponding
    models in our format.
    '''

    robotName = urdf.robotName
    links  = ODict()
    joints = ODict()
    pairs = []
    children = {}
    orphans = []

    for urdfname in urdf.links.keys() :
        name = toValidID( urdfname )
        link = robmodel.connectivity.Link(name)
        links[name] = link
        children[name] = []
        if urdf.links[urdfname].parent == None :
            orphans.append(link)

    if len(orphans)==0 :
        logger.fatal("Could not find any root link (i.e. a link without parent).")
        logger.fatal("Check for kinematic loops.")
        print("Error, no root link found. Aborting", file=sys.stderr)
        sys.exit(-1)
    if len(orphans) > 1 :
        logger.warning("Found {0} links without parent, only one expected".format(len(orphans)))
        logger.warning("Any robot model must have exactly one root element.")
        logger.warning("This might lead to unexpected results.")
    robotBase = orphans[0]

    for jname in urdf.joints.keys() :
        urdfjoint = urdf.joints[jname]
        name      = toValidID( jname )
        jkind     = urdfjoint.type
        if jkind in JointKind :
            jkind = JointKind[jkind]
        else :
            # 'jkind' remains a string
            logger.warning("Unknown joint type '{}' for joint '{}'".format(jkind, jname))

        joint     = robmodel.connectivity.Joint(name, jkind)
        parent    = links[ toValidID(urdfjoint.parent) ]
        child     = links[ toValidID(urdfjoint.child)  ]
        children[parent.name].append( child )

        joints[name] = joint
        pairs.append( robmodel.connectivity.KPair(joint, parent, child) )


    # CONNECTIVITY MODEL
    connectivityModel = robmodel.connectivity.Robot(
        urdf.robotName, links, joints, pairs)

    # REGULAR NUMBERING
    # There is no numbering scheme in the URDF format, so we arbitrarily
    # associate code to each link via a Depth-First-Traversal
    code = 0
    numbering = {}
    fixedLinks = []
    def setCode(currentLink, parent):
        nonlocal code, numbering, fixedLinks
        if currentLink == None : return

        joint = connectivityModel.linkPairToJoint(currentLink, parent)
        if joint is not None and joint.kind == JointKind.fixed :
            fixedLinks.append(currentLink)
        else:
            numbering[currentLink.name] = code
            code = code + 1
        for child in children[currentLink.name] :
            setCode( child, currentLink )

    setCode( robotBase, None)
    for fl in fixedLinks :
        numbering[fl.name] = code
        code = code + 1

    ordering = { 'robot': robotName, 'nums' : numbering }
    orderedModel = robmodel.ordering.Robot(connectivityModel, ordering)

    # FRAMES
    # The URDF does not have explicit frames, so there are no more frames than
    # joints and links; thus the second argument is always the empty list
    framesModel = robmodel.frames.RobotDefaultFrames(orderedModel, [])

    # GEOMETRY MEASUREMENTS
    poses = []
    axes = {}
    for joint in urdf.joints.values() :
        # Get the current joint and link (predecessor)
        myjoint = joints[ toValidID( joint.name ) ]
        mylink  = orderedModel.predecessor(myjoint)

        logger.debug("Processing joint * {0} * and predecessor link * {1} *".format(myjoint.name, mylink.name) )

        # The relative pose of the URDF joint frame relative to the URDF link frame
        xyz, rpy, motion_link_to_joint = linkFrameToJointFrameInURDF(joint)

        jaxis = np.round( np.array(joint.frame['axis']), 5)

        logger.debug("Joint axis in URDF coordinates    : {0}".format(jaxis) )
        logger.debug("URDF joint xyz and rpy attributes : {0}   {1}".format(xyz, rpy) )

        frame_joint = framesModel.framesByName[ robmodel.frames.jointFrameName(orderedModel, myjoint) ]
        frame_link  = framesModel.framesByName[ robmodel.frames.linkFrameName(orderedModel, mylink)  ]
        pose = primitives.Pose(target=frame_joint, reference=frame_link)
        poses.append( PoseSpec(pose, motion_link_to_joint) )
        axes[myjoint.name] = joint.frame['axis']

    posesModel = motions.PosesSpec(robotName, poses)

    geometryModel = robmodel.geometry.Geometry(orderedModel, framesModel, posesModel, axes)
    return connectivityModel, orderedModel, framesModel, geometryModel
