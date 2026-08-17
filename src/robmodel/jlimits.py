from dataclasses import dataclass
from robmodel import logger
from robmodel.connectivity import JointKind

@dataclass
class JointLimit:
    lower_pos : float = 0
    upper_pos : float = 0
    velocity  : float = 0
    force     : float = 0


class JointLimits:
    def __init__(self, connectivity_model, ranges_by_name):
        '''
        - `ranges_by_name` dictionary of dictionaries: the first must be keyed
           by joint name, and every value must be unpackable to a `JointLimit`
           instance, i.e., it must be possible to do
           `JointLimit(**ranges_by_name[someJointName]) `
        '''
        self.robot = connectivity_model
        self.limits = {}

        for jname,joint in connectivity_model.joints.items():
            if joint.kind != JointKind.fixed:
                if jname not in ranges_by_name:
                    logger.warning("JointLimits: no limits data for joint '%s'", jname)

        for jname, limdata in ranges_by_name.items():
            if jname in self.robot.joints:
                self.limits[ self.robot.joints[jname] ] = JointLimit( **limdata )
            else:
                logger.warning("JointLimits: joint '%s' does not belong to robot '%s'", jname, self.robot.name)

    @property
    def byJoint(self):
        return self.limits

    @property
    def connectivityModel(self):
        '''
        The connectivity model this instance was constructed with
        '''
        return self.robot

