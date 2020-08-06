'''
The inertial data of a robot model
'''

import logging

logger = logging.getLogger(__name__)

class CoM:
    def __init__(self, frame, x=0, y=0, z=0):
        self.frame = frame
        self.x = x
        self.y = y
        self.z = z

class IMoments:
    def __init__(self, frame, ixx=1, iyy=1, izz=1, ixy=0, ixz=0, iyz=0):
        self.frame = frame
        self.ixx = ixx
        self.iyy = iyy
        self.izz = izz
        self.ixy = ixy
        self.ixz = ixz
        self.iyz = iyz

class BodyInertia:
    def __init__(self, mass, com, moments):
        self.mass = mass
        self.com  = com
        self.moments = moments

    def __str__(self):
        return 'm: {}  com: {}  moments: {}'.format(self.mass, self.com, self.moments)


class RobotLinksInertia:
    '''
    A composition of a `robmodel.connectivity.Robot` model and inertial data
    '''

    def __init__(self, robotConnectivity, robotFrames, inertias):
        '''
        Constructor arguments:

        - `robotConnectivity`: the robot connectivity model
        - `inertias`: a dictionary with link names as keys, and `BodyInertia`
           objects as values
        '''
        if robotConnectivity.name != robotFrames.robot.name :
            raise RuntimeError("The connectivity and the frames model refer to different robots")

        self.robot   = robotConnectivity
        self.frames  = robotFrames
        self.inertia = {}
        for linkName in robotConnectivity.links :
            if linkName not in inertias :
                logger.warning("Inertia for link '{l}' is missing".format(l=linkName))
            else :
                self.inertia[linkName] = inertias[linkName]

        for linkName in inertias :
            if linkName not in robotConnectivity.links :
                logger.warning("Inertia given for link '{l}', which does not appear in the robot model '{r}'".format(l=linkName, r=robotConnectivity.name))


    def _checkGivenInertia(self, inip):
        com = inip.com.copy()    # a shallow copy
        im  = inip.moments.copy()

        com_frame = self._checkLinkFrame(inip.com.frame)
        if com_frame is None :
            logger.warning("Could not validate CoM frame - data might be inconsistent")
        else :
            com.frame = com_frame  # make sure to use the validated link frame

        moments_frame = self._checkLinkFrame(inip.moments.frame)
        if moments_frame is None :
            logger.warning("Could not validate inertia moments frame - data might be inconsistent")
        else :
            im.frame = moments_frame

        return BodyInertia(mass=inip.mass, com=com, moments=im)


    def _checkLinkFrame(self, frame):
        if isinstance(frame, str) :  # it is the _name_ of a frame
            if frame in self.frames.linkFrames :
                return self.frames.linkFrames[frame] # from name to actual object
            else :
                logger.error("Unknown link frame '{f}' in robot '{r}'".format(frame, self.robot.name))
        else :
            if frame in self.frames.linkFrames.values() :
                return frame
            else :
                logger.error("The given argument is not a link frame of robot '{r}'".format(self.robot.name))
        return None




