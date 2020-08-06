import re, numbers
import yaml
import sympy

from kgprim.core import Frame
from kgprim.core import Pose
import kgprim.motions as motions
from kgprim.motions import MotionStep
from kgprim.motions import MotionSequence
from kgprim.motions import PoseSpec
from kgprim.motions import PosesSpec
import kgprim.values as expr
import robmodel.connectivity    as robot_connectivity
import robmodel.ordering as robot_ordering
import robmodel.inertia  as bodyinertia


# Matches "pi" case insensitive, with some other possible non-letter
# characters around, like "pi/2". It captures the "pi"
pimatcher = re.compile("[^a-zA-Z]*([P|p][I|i])[^a-zA-Z]*")


def connectivity(istream):
    data  = yaml.safe_load(istream)
    return robot_connectivity.fromDict(data)


def numbering_scheme(istream):
    data  = yaml.safe_load(istream)
    return robot_ordering.numberingSchemeFromDict(data)



def inertia(istream, floatLiteralsAsConstants=False):
    data = yaml.safe_load(istream)
    if 'inertia' not in data :
        logger.error("YAML data must have an 'inertia' element")
        return None
    data = data['inertia']
    ret = {}
    for link in data.keys() :
        current = data[link]
        mass = _getPropertyValue(current['mass'], floatLiteralsAsConstants)

        com = current['com']  # defaults to the same dictionary as in the YAML
        for v in ['x', 'y', 'z'] :
            if v in com :
                # however, we possibly overwrite some values here
                com[v] = _getPropertyValue(com[v], floatLiteralsAsConstants)
        com  = bodyinertia.CoM( **com )

        im = current['moments']
        for v in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'] :
            if v in im :
                im[v] = _getPropertyValue(im[v], floatLiteralsAsConstants)
        im   = bodyinertia.IMoments( **im )

        ret[link] = bodyinertia.BodyInertia(mass=mass, com=com, moments=im)

    return ret



def geometry(istream, floatLiteralsAsConstants=False) :
    data = yaml.safe_load(istream)
    out = {}

    def makeMotionStep( dictdata ):
        kind_axis = makeMotionStep.d[ dictdata['kind'] ]
        amount    = _getPropertyValue(dictdata['amount'], floatLiteralsAsConstants)
        return MotionStep(kind_axis[0], kind_axis[1], amount)

    makeMotionStep.d = {
        'trx' : (MotionStep.Kind.Translation, motions.Axis.X),
        'try' : (MotionStep.Kind.Translation, motions.Axis.Y),
        'trz' : (MotionStep.Kind.Translation, motions.Axis.Z),
        'rotx': (MotionStep.Kind.Rotation  , motions.Axis.X),
        'roty': (MotionStep.Kind.Rotation  , motions.Axis.Y),
        'rotz': (MotionStep.Kind.Rotation  , motions.Axis.Z),
    }


    modeStr = data['mode']
    if not modeStr in MotionSequence.Mode.__members__ :
        raise ValueError("Unknown mode '{0}'; only '{1}' and '{2}' are valid"
                        .format(modeStr, MotionSequence.Mode.currentFrame.name,
                                      MotionSequence.Mode.fixedFrame.name) )
    mode = MotionSequence.Mode[modeStr]
    poses = []
    srcMotions = data['poses']
    for srcMotion in srcMotions :
        tgt    = Frame(srcMotion['target'])
        ref    = Frame(srcMotion['reference'])
        pose   = Pose(target=tgt, reference=ref)
        steps  = [makeMotionStep( srcStep ) for srcStep in srcMotion['steps'] ]
        motion = MotionSequence(steps=steps, mode=mode)
        poses.append( PoseSpec( pose=pose, motion=motion) )

    out['name']  = data['model']
    out['poses'] = poses
    return PosesSpec(**out)



def _getPropertyValue(yamlvalue, floatLiteralsAsConstants=False):
    ret = yamlvalue
    if isinstance(yamlvalue, numbers.Real) :
        if floatLiteralsAsConstants :
            cc = expr.Constant(name="", value=yamlvalue) #TODO name?!?
            ret = expr.Expression(cc)

    elif isinstance(yamlvalue, str) :
        match = pimatcher.fullmatch(yamlvalue)
        if not match :
            raise RuntimeError("Unrecognized expression: '{0}'".format(yamlvalue))
        else :
            pi = expr.MyPI.instance()
            # Make sure to replace the case-insensitive pi string with lowercase "pi"
            exprtext = yamlvalue.replace( match.group(1), "pi" )
            ret = expr.Expression(argument=pi, sympyExpr=sympy.sympify(exprtext))
    else :
        raise RuntimeError("Unknown value '{0}'".format(str(yamlvalue)))

    return ret



