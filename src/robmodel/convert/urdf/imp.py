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
import robmodel.inertia
import robmodel.jlimits
from robmodel.connectivity import JointKind
from robmodel.jlimits import JointLimit
import dataclasses

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
            self.limits= None
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
            joint.limits= self.readJointLimitsData( nodejoint )
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
            params['moments'] = {}
            for m in URDFWrapper.iMomentsLabels :
                params['moments'][m] = 0.0
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

        # The URDF stores the elements of the tensor, not the inertia moments
        # Thus we have to swap the sign of the non-diagonal elements
        moments = paramsNode.find('inertia')
        aux = {}
        aux['ixx'] =   float(moments.get('ixx'))
        aux['iyy'] =   float(moments.get('iyy'))
        aux['izz'] =   float(moments.get('izz'))
        aux['ixy'] = - float(moments.get('ixy'))
        aux['ixz'] = - float(moments.get('ixz'))
        aux['iyz'] = - float(moments.get('iyz'))

        params['moments'] = aux
        params['mass']    = mass
        params['xyz']     = xyz
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


    def readJointLimitsData(self, jointNode):
        ret = None
        node = jointNode.find('limit')
        if node is not None :
            ret = JointLimit()
            aux = node.get('lower')
            ret.lower_pos = float(aux) if aux else None
            aux = node.get('upper')
            ret.upper_pos = float(aux) if aux else None
            aux = node.get('effort')
            ret.force = float(aux) if aux else None
            aux = node.get('velocity')
            ret.velocity = float(aux) if aux else None
        return ret


def com_frame_name(link) :
    return 'fr_' + link.name + '_com'


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



def convert( urdf, ignoreFixedJoints=False, **kwargs) :
    '''
    Reads the model from a URDFWrapper instance, and construct the corresponding
    models in our format.
    '''

    robotName = urdf.robotName
    pairs = []
    children = {}
    orphans = [name for name in urdf.links if urdf.links[name].parent == None]

    if len(orphans)==0 :
        logger.fatal("Could not find any root link (i.e. a link without parent).")
        logger.fatal("Check for kinematic loops.")
        raise RuntimeError("no root link found")
    if len(orphans) > 1 :
        logger.warning("Found {0} links without parent, only one expected".format(len(orphans)))
        logger.warning("Any robot model must have exactly one root element.")
        logger.warning("This might lead to unexpected results.")

    # Build the list of kinematic pairs by going through the sequence of joints.
    # Use a cache to avoid creating multiple times the "same" link (for those
    # appearing in multiple pairs). Although this would not normally be
    # a problem, because `Link` overrides equality, I want to make sure there is
    # only one unique instance in memory.
    linksPool = {}
    for name in urdf.joints :
        urdfjoint = urdf.joints[name]
        jkind     = urdfjoint.type
        if jkind in JointKind.__members__ :
            jkind = JointKind[jkind]
        else :
            # 'jkind' remains a string
            logger.warning("Unknown joint type '{}' for joint '{}'. Storing the string value rather than the enum item".format(jkind, jname))

        if not (jkind==JointKind.fixed and ignoreFixedJoints):
            joint  = robmodel.connectivity.Joint(name, jkind)
            parent = linksPool.setdefault(urdfjoint.parent, robmodel.connectivity.Link(urdfjoint.parent))
            child  = linksPool.setdefault(urdfjoint.child , robmodel.connectivity.Link(urdfjoint.child ))
            children.setdefault(parent.name, [])
            children[parent.name].append( child.name )
            pairs.append( robmodel.connectivity.KPair(joint, parent, child) )
        else:
            logger.info("Dropping fixed joint '{}'".format(name))

    # CONNECTIVITY MODEL
    connectivityModel = robmodel.connectivity.Robot(robotName, pairs)

    # REGULAR NUMBERING
    # There is no numbering scheme in the URDF format, so we arbitrarily
    # associate code to each link via a Depth-First-Traversal
    robotBase = orphans[0]
    code = 1
    numbering = {}
    fixedLinks = []
    def setCode(currentLink, parent):
        nonlocal code, numbering, fixedLinks
        joint = connectivityModel.linkPairToJoint(
            connectivityModel.links[currentLink], connectivityModel.links[parent])
        if joint.kind == JointKind.fixed :
            fixedLinks.append(currentLink)
        else:
            numbering[currentLink] = code
            code = code + 1
        for child in children.get(currentLink, []) :
            setCode( child, currentLink )

    numbering[robotBase] = 0
    for child in children.get(robotBase, []) :
        setCode( child, robotBase )

    if not dropFixedJoints:
        for fl in fixedLinks :
            numbering[fl] = code
            code = code + 1

    ordering = { 'robot': robotName, 'nums' : numbering }
    orderedModel = robmodel.ordering.Robot(connectivityModel, ordering)

    # FRAMES
    # The URDF does not have explicit frames, so there are no more frames than
    # joints and links.
    # However, it implicitly uses frames with origin at the CoMs, because the
    # inertial moments are defined there, according to the specs.
    comFrames = []
    for _,link in orderedModel.links.items():
        comFrames.append( primitives.Attachment( primitives.Frame(com_frame_name(link)), link ) )

    framesModel = robmodel.frames.RobotDefaultFrames(orderedModel, comFrames)

    # GEOMETRY MEASUREMENTS
    poses = []
    axes = {}
    for name, myjoint in orderedModel.joints.items() :
        mylink = orderedModel.predecessor(myjoint)
        joint  = urdf.joints[name]

        logger.debug("Processing joint * {0} * and predecessor link * {1} *".format(name, mylink.name) )

        # The relative pose of the URDF joint frame relative to the URDF link frame
        xyz, rpy, motion_link_to_joint = linkFrameToJointFrameInURDF(joint)

        jaxis = np.round( np.array(joint.frame['axis']), 5)

        logger.debug("Joint axis in URDF coordinates    : {0}".format(jaxis) )
        logger.debug("URDF joint xyz and rpy attributes : {0}   {1}".format(xyz, rpy) )

        frame_joint = framesModel.framesByName[ robmodel.frames.jointFrameName(orderedModel, myjoint) ]
        frame_link  = framesModel.framesByName[ robmodel.frames.linkFrameName(orderedModel, mylink)  ]
        pose = primitives.Pose(target=frame_joint, reference=frame_link)
        poses.append( PoseSpec(pose, motion_link_to_joint) )
        axes[name] = joint.frame['axis']

    # Add pose information for the CoM frames
    for name, link in orderedModel.links.items() :
        linkFrame = framesModel.byLink[ link ]
        comFrame  = framesModel.byName[com_frame_name(link)]

        pose = primitives.Pose(target=comFrame, reference=linkFrame)
        com  = urdf.links[name].inertia['xyz']
        tr   = [motions.translation(a, com[a.value]) for a in motions.Axis if round(com[a.value],5) != 0.0]

        poses.append( PoseSpec(pose, MotionSequence(tr, MotionSequence.Mode.fixedFrame)) )

    posesModel = motions.PosesSpec(robotName, poses)
    geometryModel = robmodel.geometry.Geometry(orderedModel, framesModel, posesModel, axes)

    # INERTIAL MODEL
    inertialData = {}
    for name, mylink in orderedModel.links.items() :
        linkFrame = framesModel.byLink[ mylink ]
        comFrame  = framesModel.byName[com_frame_name(mylink)]

        link = urdf.links[name]
        com  = link.inertia['xyz']
        com  = robmodel.inertia.CoM(linkFrame, com[0], com[1], com[2] )
        mass = link.inertia['mass']

        moments = robmodel.inertia.IMoments(comFrame, **link.inertia['moments'])

        inertialData[ name ] = robmodel.inertia.BodyInertia(mass, com, moments)
    inertiaModel = robmodel.inertia.RobotLinksInertia(connectivityModel, framesModel, inertialData)


    # JOINT LIMITS
    jlimits_data = {jname : dataclasses.asdict(urdf.joints[jname].limits)
                        for jname in orderedModel.joints }
    limitsModel = robmodel.jlimits.JointLimits(orderedModel, jlimits_data)

    return connectivityModel, orderedModel, framesModel, geometryModel, inertiaModel, limitsModel
