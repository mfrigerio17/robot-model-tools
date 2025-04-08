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
    '''
    Minimal wrapper of the textX grammar for the KinematicsDSL

    Only exposes the function to load a model from file.
    '''

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
        '''
        The runtime representation of the model hosted in the given file.
        The format is determined mostly by the grammar and by textX behavior.
        Loading is customized only as far as Constants and Parameters are
        concerned, such that "parameters" and "constants" as defined by the
        grammar, are automatically converted to objects of `kgprim.values`.
        '''
        return self.mm.model_from_file(file)




class Importer:
    '''
    Loads a KinematicsDSL runtime model, and converts it to the models in the
    `robmodel` package.
    '''

    jointTypeStr = {
        'RevoluteJoint' : robmodel.connectivity.JointKind.revolute,
        'PrismaticJoint': robmodel.connectivity.JointKind.prismatic
        }

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

    def __init__(self, floatLiteralsAsConstants=False):
        self.floatsAsConsts = floatLiteralsAsConstants

    def convert(self, kinDSLModel):
        links  = ODict()
        joints = ODict()
        kpairs = []

        allBodies = [kinDSLModel.base] + kinDSLModel.links

        for linkin in allBodies:
            name = linkin.name
            link = robmodel.connectivity.Link(name)
            links[name] = link

        for jointin in kinDSLModel.joints :
            name  = jointin.name
            joint = robmodel.connectivity.Joint(name, self.jointTypeStr[jointin.__class__.__name__])
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
                        kinDSLModel.name, links, joints, kpairs)

        # NUMBERING SCHEME
        numbering = {}
        numbering[ kinDSLModel.base.name ] = 0
        for linkin in kinDSLModel.links :
            numbering[linkin.name] = linkin.num

        ordering = { 'robot': kinDSLModel.name, 'nums' : numbering }
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
        for jointin in kinDSLModel.joints :
            motion_link_to_joint = self._motion_linkFrameToOtherFrame(jointin.refFrame, jointin.name)
            joint = joints[ jointin.name ]
            link  = orderedModel.predecessor(joint)

            frame_joint = framesModel.framesByName[ robmodel.frames.jointFrameName(orderedModel, joint) ]
            frame_link  = framesModel.framesByName[ robmodel.frames.linkFrameName(orderedModel, link) ]
            pose = primitives.Pose(target=frame_joint, reference=frame_link)
            poses.append( PoseSpec(pose, motion_link_to_joint) )

        for linkin in allBodies :
            for fr in linkin.frames :
                motion_link_to_user = self._motion_linkFrameToOtherFrame(fr.transform, fr.name)
                frame_link = framesModel.framesByName[ robmodel.frames.linkFrameName(orderedModel, linkin) ]
                frame_user = framesModel.framesByName[ fr.name ]
                pose       = primitives.Pose(target=frame_user, reference=frame_link)
                poses.append( PoseSpec(pose, motion_link_to_user) )

        posesModel = motions.PosesSpec(kinDSLModel.name, poses)
        geometryModel = robmodel.geometry.Geometry(orderedModel, framesModel, posesModel)

        # INERTIA
        inertiadict = {}
        inertiaBodies = kinDSLModel.links
        if kinDSLModel.base.__class__.__name__ == 'FloatingRobotBase' :
            inertiaBodies = allBodies

        for linkin in inertiaBodies:
            ipin = linkin.bodyInertia
            if ipin is not None:
                #TODO support the inertia properties in the user-frame
                # Otherwise, the KinDSL format uses the default link frame for all
                # the inertia properties.
                frame = framesModel.framesByName[ robmodel.frames.linkFrameName(orderedModel, linkin) ]
                mass = self._exprValue(ipin.mass, linkin.name+'_mass')
                com = inertia.CoM(frame,
                                  x = self._exprValue(ipin.com.x, linkin.name+'_comx'),
                                  y = self._exprValue(ipin.com.y, linkin.name+'_comy'),
                                  z = self._exprValue(ipin.com.z, linkin.name+'_comz'))
                im = inertia.IMoments(frame,
                                      ixx = self._exprValue(ipin.ixx, linkin.name+'_ixx'),
                                      iyy = self._exprValue(ipin.iyy, linkin.name+'_iyy'),
                                      izz = self._exprValue(ipin.izz, linkin.name+'_izz'),
                                      ixy = self._exprValue(ipin.ixy, linkin.name+'_ixy'),
                                      ixz = self._exprValue(ipin.ixz, linkin.name+'_ixz'),
                                      iyz = self._exprValue(ipin.iyz, linkin.name+'_iyz'))

                inertiadict[linkin.name] = inertia.BodyInertia(mass, com, im)

        inertiaModel = None
        if len(inertiadict) > 0:
            inertiaModel = inertia.RobotLinksInertia(connectivityModel, framesModel, inertiadict)


        return connectivityModel, orderedModel, framesModel, geometryModel, inertiaModel


    def _motion_linkFrameToOtherFrame(self, otherFrame, frameName):
        tr  = otherFrame.translation
        rot = otherFrame.rotation

        ids    = [frameName + suffix for suffix in ["_tx","_ty","_tz"]]
        trval  = [self._exprValue(t[0],t[1]) for t in zip([tr.x, tr.y, tr.z],ids)]
        ids    = [frameName + suffix for suffix in ["_rx","_ry","_rz"]]
        rotval = [self._exprValue(r[0],r[1]) for r in zip([rot.x, rot.y, rot.z],ids)]

        tr = [motions.translation(a, trval[a.value])  for a in motions.Axis if trval[a.value] != 0.0]
        rt = [motions.rotation   (a, rotval[a.value]) for a in motions.Axis if rotval[a.value]!= 0.0]

        motion__linkToJoint = MotionSequence(tr+rt, MotionSequence.Mode.currentFrame)
        return motion__linkToJoint


    def _exprValue(self, expr, propertyName=""):
        cname = expr.__class__.__name__

        if cname not in self.exprvalue_float :
            raise RuntimeError("Unsupported property type ({0}) in the input KinDSL model".format(cname))
        if isinstance(expr.arg, float) :
            fvalue = self.exprvalue_float[ cname ]( expr )
            if round(fvalue,5) == 0.0 :  #TODO magic number 5
                return 0.0  # regardless of the option, 0 never becomes a named constant
            if self.floatsAsConsts:
                cvalue = vpc.Constant(name=propertyName, value=fvalue)
                return vpc.Expression(cvalue)
            else:
                return fvalue
        else :
            return self.exprvalue_symb[ cname ]( expr )






def convert(kindslFile, floatLiteralsAsConstants=False):
    dsl = KinematicsDSL()
    robin = dsl.modelFromFile(kindslFile)
    converter = Importer(floatLiteralsAsConstants=floatLiteralsAsConstants)
    return converter.convert(robin)



