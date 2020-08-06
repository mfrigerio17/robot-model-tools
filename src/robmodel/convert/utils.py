import math

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
        # rstrip remove the trailing zeros, if any
        return( ( "{" + self.formatStr + "}" ).format( num ).rstrip('0') )



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
