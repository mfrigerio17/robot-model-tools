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
    urdfFrames = []

    for urdfname in urdf.links.keys() :
        name = toValidID( urdfname )
        link = robmodel.connectivity.Link(name)
        links[name] = link
        children[name] = []
        if urdf.links[urdfname].parent == None :
            orphans.append(link)

        #link_R_urdflink[name] = np.identity(3) # initial value

        fr = primitives.Frame("urdf_" + link.name)
        urdfFrames.append( primitives.Attachment(fr, link) )

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
        if jkind in JointKind.__members__.keys() :
            jkind = JointKind[jkind] # otherwise, it stays a string (like 'fixed')
        joint     = robmodel.connectivity.Joint(name, jkind)
        parent    = links[ toValidID(urdfjoint.parent) ]
        child     = links[ toValidID(urdfjoint.child)  ]
        children[parent.name].append( child )

        joints[name] = joint
        pairs.append( robmodel.connectivity.KPair(joint, parent, child) )

        fr = primitives.Frame("urdf_" + joint.name)
        urdfFrames.append( primitives.Attachment(fr, parent) )

    # CONNECTIVITY MODEL
    connectivityModel = robmodel.connectivity.Robot(
        urdf.robotName, links, joints, pairs)

    # REGULAR NUMBERING
    # There is no numbering scheme in the URDF format, so we arbitrarily
    # associate code to each link via a Depth-First-Traversal
    code = 0
    numbering = {}
    def setCode(currentLink):
        nonlocal code, numbering
        if currentLink == None : return
        numbering[currentLink.name] = code
        for child in children[currentLink.name] :
            code = code + 1
            setCode( child )

    setCode( robotBase )
    ordering = { 'robot': robotName, 'nums' : numbering }
    orderedModel = robmodel.ordering.Robot(connectivityModel, ordering)

    # FRAMES
    framesModel = robmodel.frames.RobotDefaultFrames(orderedModel, urdfFrames)

    # GEOMETRY MEASUREMENTS
    # For each link, the motion to go from the link frame in our convention to
    # the URDF frame of the same link. There might be differences because our
    # model still requires the Z axis of the link frame to be aligned with the
    # joint axis. Initialize with empty list.
    M_link_urdflink = {}
    for link in links.values() :
        M_link_urdflink[link.name] = MotionSequence([], MotionSequence.Mode.currentFrame)

    poses = []
    for joint in urdf.joints.values() :
        # Get the current joint and link (predecessor)
        myjoint = joints[ toValidID( joint.name ) ]
        mylink  = orderedModel.predecessor(myjoint)
        motion_link_to_urdflink = M_link_urdflink[mylink.name]

        logger.debug("Processing joint * {0} * and predecessor link * {1} *".format(myjoint.name, mylink.name) )
        logger.debug("Motion from link frame to URDF link frame for * {0} *: {1}".format(mylink.name, motion_link_to_urdflink))

        # The relative pose of the URDF joint frame relative to the URDF link frame
        xyz, rpy, motion_urdflink_to_urdfjoint = linkFrameToJointFrameInURDF(joint)

        # This is the motion to go from our link frame to the URDF joint frame
        motion_link_to_urdfjoint = MotionPath([motion_link_to_urdflink, motion_urdflink_to_urdfjoint])

        jaxis = np.round( np.array(joint.frame['axis']), 5)

        logger.debug("Joint axis in URDF coordinates    : {0}".format(jaxis) )
        logger.debug("URDF joint xyz and rpy attributes : {0}   {1}".format(xyz, rpy) )

        # If the joint axis is not the Z axis
        if not np.equal(jaxis, np.array((0.,0.,1.))).all() and joint.type != "fixed" :
            logger.info("The axis of joint '{0}' is not the Z axis".format(joint.name) )

            x,y,z = jaxis[0], jaxis[1], jaxis[2]
            rz = - math.atan2(x,y)
            rx = - math.atan2( math.sqrt(x*x + y*y), z )
            rotations = []
            if round(rz,5) != 0.0 :
                rotations.append( motions.rotation(motions.Axis.Z, rz) )
            if round(rx,5) != 0.0 :
                rotations.append( motions.rotation(motions.Axis.X, rx) )

            rotation_urdfjoint_to_alignZ = MotionSequence(rotations, MotionSequence.Mode.currentFrame)

            logger.debug("Rotations of the link frame required to align the Z axis with the joint axis: {0}".format( rotations ))

            motion_link_to_joint = MotionPath([motion_link_to_urdfjoint, rotation_urdfjoint_to_alignZ])

            successor = orderedModel.successor(myjoint)
            M_link_urdflink[successor.name] = motions.reverse(rotation_urdfjoint_to_alignZ)

        else :
            motion_link_to_joint = motion_link_to_urdfjoint

        frame_joint = framesModel.framesByName[ myjoint.name ]
        frame_link  = framesModel.framesByName[ mylink.name  ]
        pose = primitives.Pose(target=frame_joint, reference=frame_link)
        poses.append( PoseSpec(pose, motion_link_to_joint) )

    posesModel = motions.PosesSpec(robotName, poses)

    geometryModel = robmodel.geometry.Geometry(orderedModel, framesModel, posesModel)
    return connectivityModel, orderedModel, framesModel, geometryModel
