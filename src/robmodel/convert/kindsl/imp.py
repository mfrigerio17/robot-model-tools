import os, math
from collections import OrderedDict as ODict
import textx


import kgprim.core as primitives
import kgprim.values as vpc
import robmodel.connectivity
import robmodel.ordering
import robmodel.frames
import robmodel.geometry
import robmodel.inertia as inertia

import kgprim.motions as motions
from kgprim.motions import MotionStep as MotionStep
from kgprim.motions import MotionSequence as MotionSequence
from kgprim.motions import PoseSpec as PoseSpec



class KinematicsDSL:
    def __init__(self):
        here = os.path.dirname(os.path.abspath(__file__))
        self.mm = textx.metamodel_from_file(here+"/kindsl.tx")
        obj_processors = {
            'PILiteral': lambda __ : vpc.MyPI.instance(),
            'Parameter': lambda x : vpc.Parameter(name=x.name, defValue=x.defvalue),
            'ExplicitConstant' : lambda x : vpc.Constant(name=x.name, value=x.value),
            'RefToConstant'    : lambda x : vpc.Constant(name=x.actual.name, value=x.actual.value)
        }
        self.mm.register_obj_processors( obj_processors )

    def modelFromFile(self, file):
        return self.mm.model_from_file(file)


dsl = KinematicsDSL()


jointTypeStr = {
    'RevoluteJoint' : robmodel.connectivity.JointKind.revolute,
    'PrismaticJoint': robmodel.connectivity.JointKind.prismatic
    }

def __raise(name):
    raise RuntimeError("Cannot get the numeric value of parameter '{0}'".format(name))

__currentParams = {}



exprvalue_float = {
    'PlainExpr' : lambda e: (-1 if e.minus else 1) * e.arg,
    'MultExpr'  : lambda e: e.mult * e.arg,
    'DivExpr'   : lambda e: (-1 if e.minus else 1)* e.arg / e.div
}
exprvalue_symb = {
    'PlainExpr' : lambda e: vpc.Expression(e.arg, (-1 if e.minus else 1) * e.arg.symbol),
    'MultExpr'  : lambda e: vpc.Expression(e.arg, e.mult * e.arg.symbol),
    'DivExpr'   : lambda e: vpc.Expression(e.arg, (-1 if e.minus else 1)* e.arg.symbol / e.div )
}

def exprValue(expr):
    cname = expr.__class__.__name__
    if cname not in exprvalue_float :
        raise RuntimeError("Unsupported property type ({0}) in the input KinDSL model".format(cname))
    if isinstance(expr.arg, float) :
        fvalue = exprvalue_float[ cname ]( expr )
        if round(fvalue,5) == 0.0 :  #TODO magic number 5
            fvalue = 0.0
        return fvalue
    else :
        return exprvalue_symb[ cname ]( expr )

def linkFrameToOtherFrameInKinDSL(refFrame):
    tr  = refFrame.translation
    rot = refFrame.rotation

    trval  = [exprValue(t) for t in [tr.x, tr.y, tr.z]]
    rotval = [exprValue(r) for r in [rot.x, rot.y, rot.z]]

    tr = [motions.translation(a, trval[a.value])  for a in motions.Axis if trval[a.value] != 0.0]
    rt = [motions.rotation   (a, rotval[a.value]) for a in motions.Axis if rotval[a.value]!= 0.0]

    motion__linkToJoint = MotionSequence(tr+rt, MotionSequence.Mode.currentFrame)
    return motion__linkToJoint


def convert(kindslFile, params={}):
    global __currentParams
    __currentParams = params
    robin = dsl.modelFromFile(kindslFile)

    links  = ODict()
    joints = ODict()
    kpairs = []

    allBodies = [robin.base] + robin.links

    for linkin in allBodies:
        name = linkin.name
        link = robmodel.connectivity.Link(name)
        links[name] = link

    for jointin in robin.joints :
        name  = jointin.name
        joint = robmodel.connectivity.Joint(name, jointTypeStr[jointin.__class__.__name__])
        joints[name] = joint

    for parentin in allBodies :
        for childSpec in parentin.childrenList.children :
            childin = childSpec.link
            jointin = childSpec.joint

            parent = links[ parentin.name ]
            child  = links[ childin.name ]
            joint  = joints[ jointin.name ]

            kpairs.append( robmodel.connectivity.KPair(joint, parent, child) )

    connectivityModel = robmodel.connectivity.Robot(
                    robin.name, links, joints, kpairs)

    # NUMBERING SCHEME
    numbering = {}
    numbering[ robin.base.name ] = 0
    for linkin in robin.links :
        numbering[linkin.name] = linkin.num

    ordering = { 'robot': robin.name, 'nums' : numbering }
    orderedModel = robmodel.ordering.Robot(connectivityModel, ordering)

    # FRAMES and POSES
    userFrames = []
    for linkin in allBodies :
        for fr in linkin.frames :
            frame = primitives.Frame(fr.name)
            attach= primitives.Attachment(entity=frame, body=links[linkin.name])
            userFrames.append( attach )
    framesModel = robmodel.frames.RobotDefaultFrames(orderedModel, userFrames)

    poses = []
    for jointin in robin.joints :
        motion_link_to_joint = linkFrameToOtherFrameInKinDSL(jointin.refFrame)
        joint = joints[ jointin.name ]
        link  = orderedModel.predecessor(joint)

        frame_joint = framesModel.framesByName[ joint.name ]
        frame_link  = framesModel.framesByName[ link.name  ]
        pose = primitives.Pose(target=frame_joint, reference=frame_link)
        poses.append( PoseSpec(pose, motion_link_to_joint) )

    for linkin in allBodies :
        for fr in linkin.frames :
            motion_link_to_user = linkFrameToOtherFrameInKinDSL(fr.transform)
            frame_link = framesModel.framesByName[ linkin.name ]
            frame_user = framesModel.framesByName[ fr.name ]
            pose       = primitives.Pose(target=frame_user, reference=frame_link)
            poses.append( PoseSpec(pose, motion_link_to_user) )

    posesModel = motions.PosesSpec(robin.name, poses)
    geometryModel = robmodel.geometry.Geometry(orderedModel, framesModel, posesModel)

    inertiadict = {}
    inertiaBodies = robin.links
    if robin.base.__class__.__name__ == 'FloatingRobotBase' :
        inertiaBodies = allBodies

    for linkin in inertiaBodies:
        ipin = linkin.bodyInertia
        #TODO support the inertia properties in the user-frame
        frame = framesModel.framesByName[ linkin.name ]
        mass = exprValue(ipin.mass)
        com = inertia.CoM(frame,
                          x = exprValue(ipin.com.x),
                          y = exprValue(ipin.com.y),
                          z = exprValue(ipin.com.z))
        im = inertia.IMoments(frame,
                              ixx = exprValue(ipin.ixx),
                              iyy = exprValue(ipin.iyy),
                              izz = exprValue(ipin.izz),
                              ixy = exprValue(ipin.ixy),
                              ixz = exprValue(ipin.ixz),
                              iyz = exprValue(ipin.iyz))

        inertiadict[linkin.name] = inertia.BodyInertia(mass, com, im)

    inertiaModel = inertia.RobotLinksInertia(connectivityModel, framesModel, inertiadict)


    return connectivityModel, orderedModel, framesModel, geometryModel, inertiaModel


