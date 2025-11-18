import math
import numpy as np
import robmodel.inertia

class FloatsFormatter :
    def __init__(self, round_digits=6, pi_round_digits=5, pi_string="pi"):
        self.round_decimals = round_digits
        self.pi_round_decimals = pi_round_digits
        self.pi_string = pi_string

        self.roundedPI     = round(math.pi,   self.pi_round_decimals)
        self.roundedHalfPI = round(math.pi/2, self.pi_round_decimals)

        self.formatStr = ':.' + str(self.round_decimals) + 'f'

    def float2str(self, num, angle=False ) :
        if angle :
            value = round(num, self.pi_round_decimals)
            sign  = "-" if value<0 else ""
            if abs(value) == self.roundedPI :
                return sign + self.pi_string
            if abs(value) == self.roundedHalfPI :
                return sign + self.pi_string + "/2.0"

        num = round(num, self.round_decimals)
        num += 0  # this trick avoids the annoying '-0.0' (minus zero)

        # 'rstrip' removes trailing zeros, if any.
        ret = ("{" + self.formatStr + "}" ).format( num ).rstrip('0')
        # but I do want at least one of them, for integers, so that it will
        #  always look like a float also to limited parsers ('N.' --> 'N.0')
        if ret.endswith('.'):
            ret = ret + '0'
        return ret



def getIntrinsicXYZFromR( R ) :
    '''
    Extract the intrinsic (rotations about rotating axes) Euler angles XYZ from
    the rotation matrix **base_R_rotated**
    '''
    rx = math.atan2(-R[1,2], R[2,2])
    ry = math.asin( R[0,2] )
    rz = math.atan2(-R[0,1], R[0,0])

    return rx, ry, rz

def intrinsic2extrinsic_XYZ(irx, iry, irz):
    sx = math.sin(irx)
    cx = math.cos(irx)
    sy = math.sin(iry)
    cy = math.cos(iry)
    sz = math.sin(irz)
    cz = math.cos(irz)

    erx = math.atan2(cx*sy*sz + sx*cz, cx*cy)
    ery = math.asin(cx*sy*cz - sx*sz)
    erz = math.atan2(cx*sz+sx*sy*cz, cy*cz)

    return erx, ery, erz


def translateInertiaMoments(inertiaMoments, mass, tr_com, tr=None):
    ixx = inertiaMoments.ixx
    iyy = inertiaMoments.iyy
    izz = inertiaMoments.izz
    ixy = inertiaMoments.ixy
    ixz = inertiaMoments.ixz
    iyz = inertiaMoments.iyz
    tensor = np.array( [[ ixx, -ixy, -ixz],
                        [-ixy,  iyy, -iyz],
                        [-ixy, -iyz,  izz] ])

    com_x = __cross_mx(tr_com)
    if tr is None:
        tensor = tensor - mass * np.matmul(com_x, com_x.T)
    else:
        vec_x = __cross_mx(tr)
        tensor = tensor - mass * (np.matmul(com_x, com_x.T) - np.matmul(vec_x, vec_x.T))

    ret = robmodel.inertia.IMoments(frame=None)
    ret.ixx =  tensor[0,0]
    ret.iyy =  tensor[1,1]
    ret.izz =  tensor[2,2]
    ret.ixy = -tensor[0,1]
    ret.ixz = -tensor[0,2]
    ret.iyz = -tensor[1,2]
    return ret


def rotoTranslateInertiaMoments(moments, mass, p_com, p_desired, current_R_desired) :
    '''
    Roto-translate the given inertia moments so that they are relative to
    another frame.

    Arguments:
      - moments: a `robmodel.inertia.IMoments` object. Note that its "frame"
        attribute is ignored in this routine.
      - mass: the mass of the same rigid body the moments refer to.
      - p_com: position of the CoM relative to the origin of the current frame,
        in current frame coordinates.
      - p_desired: position of the new origin relative to the origin of the
        current frame, in current frame coordinates
      - current_R_desired: the 3x3 rotation matrix that rotates from the desired
        frame to the current frame coordinates

    Note that, as always, I am expecting `robmodel.inertia.IMoments` to be the
    actual moments of inertia, NOT the elements of the tensor: the off-diagonal
    terms have opposite sign
    '''

    # This is the position of the CoM relative to the NEW origin, in CURRENT
    # frame coordinates:
    vec = p_com[0:3] - p_desired[0:3]

    # Parallel-axis theorem to translate the moments. Note that the compact
    # formula uses the tensor
    com_x = __cross_mx(p_com)
    vec_x = __cross_mx(vec)

    tensor = np.array( [[ moments.ixx, -moments.ixy, -moments.ixz],
                        [-moments.ixy,  moments.iyy, -moments.iyz],
                        [-moments.ixz, -moments.iyz,  moments.izz] ])
    tensor = tensor - mass * (np.matmul(com_x, com_x.T) - np.matmul(vec_x, vec_x.T))

    # Rotation
    tensor2 = np.matmul(np.matmul(current_R_desired.T, tensor), current_R_desired)

    ret = robmodel.inertia.IMoments(frame=None)
    ret.ixx =  tensor2[0,0]
    ret.iyy =  tensor2[1,1]
    ret.izz =  tensor2[2,2]
    ret.ixy = -tensor2[0,1]
    ret.ixz = -tensor2[0,2]
    ret.iyz = -tensor2[1,2]
    return ret


def __cross_mx(r) :
    return np.array(
        [[ 0   , -r[2],  r[1] ],
         [ r[2],   0  , -r[0] ],
         [-r[1],  r[0],    0  ]] )
