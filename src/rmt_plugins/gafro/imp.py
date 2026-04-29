import yaml, math
import numpy as np

import kgprim.core as primitives
import kgprim.motions as motions
from kgprim.motions import MotionSequence
from kgprim.motions import PoseSpec
from kgprim.motions import MotionPath

import robmodel.connectivity
import robmodel.ordering
from robmodel.connectivity import Link
from robmodel.connectivity import Joint
from robmodel.connectivity import JointKind
from robmodel.connectivity import KPair

from robmodel.convert.urdf.imp import com_frame_name # my default name for the implicit frames at the CoM

from rmt_plugins.gafro import logger


def linkFrameToJointFrameMotion(gafrojoint):
    '''
    Return the data about the location of the joint frame relative to the
    predecessor link frame.

    This function returns
    - the rigid motion model representing the relative pose (this is the motion
      that the link frame should undergo to coincide with the joint frame)
    '''
    tf   = gafrojoint["transform"]
    xyz  = tf["translation"]
    quat = tf["rotation"]
    qw = float(quat["w"])
    qx = float(quat["x"])
    qy = float(quat["y"])
    qz = float(quat["z"])
    trv = {
        motions.Axis.X : round(float(xyz["x"]), 5),
        motions.Axis.Y : round(float(xyz["y"]), 5),
        motions.Axis.Z : round(float(xyz["z"]), 5)
    }
    rpy = {
    # quaternion to _intrinsic_ rotations (i.e. rotations about rotating
    # axes)
        motions.Axis.X : round(math.atan2( 2* (qw*qx+qy*qz), 1-2*(qx*qx+qy*qy) ), 5),
        motions.Axis.Y : round(math.asin ( 2* (qw*qy - qx*qz) ), 5),
        motions.Axis.Z : round(math.atan2( 2* (qw*qz+qx*qy), 1-2*(qz*qz+qy*qy) ), 5)
    }
    tr = [motions.translation(a, trv[a]) for a in motions.Axis if trv[a] != 0.0]
    rt = [motions.rotation   (a, rpy[a]) for a in motions.Axis if rpy[a] != 0.0]

    motion__linkToJoint = MotionSequence(tr+rt, MotionSequence.Mode.currentFrame)
    return motion__linkToJoint


def import_model(istream, dropFixedJoints=False, **kwargs):
    data = yaml.safe_load(istream)
    sysdata    = data["system"]
    src_joints = data["joints"]
    src_links  = data["links"]
    robotName  = sysdata["name"]
    children = {}
    pairs = []

    linksPool = {}
    for name, dic in src_joints.items():
        name2 = dic.get("name", name)
        if name != name2 :
            logger.warning("Inconsistent naming in the source model")
        jkind = JointKind[dic.get("type", JointKind.fixed.name)]
        if not (jkind==JointKind.fixed and dropFixedJoints):
            joint  = robmodel.connectivity.Joint(name=name2, kind=jkind)
            namep  = dic["parent_link"]
            namec  = dic["child_link"]
            parent = linksPool.setdefault(namep, Link(namep))
            child  = linksPool.setdefault(namec, Link(namec))
            children.setdefault(parent.name, [])
            children[parent.name].append( child.name )
            pairs.append( robmodel.connectivity.KPair(joint, parent, child) )
        else:
            logger.info("Dropping fixed joint '{}'".format(name))

    connectivityModel = robmodel.connectivity.Robot(name=robotName, pairs=pairs)


    # REGULAR NUMBERING
    # There is no numbering scheme in the Gafro YAML format, so we
    # arbitrarily associate code to each link via a Depth-First-Traversal
    robotBase = sysdata["root_link"]
    code = 1
    numbering = {}
    def setCode(currentLink, parent):
        nonlocal code, numbering
        joint = connectivityModel.linkPairToJoint(
            connectivityModel.links[currentLink], connectivityModel.links[parent])
        numbering[currentLink] = code
        code = code + 1
        for child in children.get(currentLink, []) :
            setCode( child, currentLink )

    numbering[robotBase] = 0
    for child in children.get(robotBase, []) :
        setCode( child, robotBase )

    ordering = { 'robot': robotName, 'nums' : numbering }
    orderedModel = robmodel.ordering.Robot(connectivityModel, ordering)

    # FRAMES
    # Gafro, like URDF, does not have explicit frames, so there are no
    # more frames than joints and links.
    # However, it implicitly uses frames with origin at the CoMs, because the
    # inertial moments are defined there
    comFrames = []
    for _,link in orderedModel.links.items():
        comFrames.append( primitives.Attachment( primitives.Frame(com_frame_name(link)), link ) )

    framesModel = robmodel.frames.RobotDefaultFrames(orderedModel, comFrames)

    # GEOMETRY MEASUREMENTS
    poses = []
    axes = {}
    for name, myjoint in orderedModel.joints.items() :
        mylink = orderedModel.predecessor(myjoint)
        joint  = src_joints[name]
        logger.debug("Processing joint * {0} * and predecessor link * {1} *".format(name, mylink.name) )

        axes[name] = tuple([round(v,5) for v in joint['axis'].values()])
        logger.debug("Joint axis in Gafro coordinates    : {0}".format(axes[name]) )

        # The relative pose of the joint frame relative to the link frame
        frame_joint = framesModel.framesByName[ robmodel.frames.jointFrameName(orderedModel, myjoint) ]
        frame_link  = framesModel.framesByName[ robmodel.frames.linkFrameName(orderedModel, mylink)  ]
        pose = primitives.Pose(target=frame_joint, reference=frame_link)
        motion_link_to_joint = linkFrameToJointFrameMotion(joint)
        poses.append( PoseSpec(pose, motion_link_to_joint) )

    # Add pose information for the implicit CoM frames
    for name, link in orderedModel.links.items() :
        linkFrame = framesModel.byLink[ link ]
        comFrame  = framesModel.byName[com_frame_name(link)]

        pose = primitives.Pose(target=comFrame, reference=linkFrame)
        link = src_links.get(name, None)
        if not link:
            logger.warning("Could not find link '%s' and its inertial properties", name)
            continue
        com  = link['inertial']['origin']['position']
        tr   = [motions.translation(a, com[c]) for a in motions.Axis if round(com[c:=a.name.lower()],5) != 0.0]

        poses.append( PoseSpec(pose, MotionSequence(tr, MotionSequence.Mode.fixedFrame)) )

    posesModel = motions.PosesSpec(robotName, poses)
    geometryModel = robmodel.geometry.Geometry(orderedModel, framesModel, posesModel, axes)

    # INERTIAL MODEL
    inertialData = {}
    for name, mylink in orderedModel.links.items() :
        linkFrame = framesModel.byLink[ mylink ]
        comFrame  = framesModel.byName[com_frame_name(mylink)]

        link = src_links.get(name, None)
        if not link:
            logger.warning("Could not find link '%s' and its inertial properties", name)
            continue
        srcData = link['inertial']
        com  = srcData['origin']['position']
        com  = robmodel.inertia.CoM(linkFrame, com['x'], com['y'], com['z'] )
        mass = srcData['mass']
        moments = srcData['inertia']
        moments = robmodel.inertia.IMoments(comFrame,
            ixx=moments['xx'], iyy=moments['yy'], izz=moments['zz'],
            ixy=-moments['xy'], ixz=-moments['xz'], iyz=-moments['yz'] )

        inertialData[ name ] = robmodel.inertia.BodyInertia(mass, com, moments)
    inertiaModel = robmodel.inertia.RobotLinksInertia(connectivityModel, framesModel, inertialData)

    # JOINT LIMITS
    convertGafroLimits = lambda glim : {
        'lower_pos': glim['lower'],
        'upper_pos': glim['upper'],
        'velocity' : glim['velocity'],
        'force'    : glim['effort']
    }
    jlimits_data = {}
    for jname in orderedModel.joints:
        glim = src_joints[jname].get('limits')
        if glim:
            jlimits_data[jname] = convertGafroLimits(glim)

    limitsModel = robmodel.jlimits.JointLimits(orderedModel, jlimits_data)

    return connectivityModel, orderedModel, framesModel, geometryModel, inertiaModel, limitsModel


