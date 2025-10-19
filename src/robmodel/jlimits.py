from dataclasses import dataclass

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

        for jname in ranges_by_name:
            if jname not in self.robot.joints:
                raise RuntimeError("cannot find joint '{}' in robot {}".format(jname, self.robot.name))

            joint  = self.robot.joints[jname]
            limits = ranges_by_name[jname]
            self.limits[joint] = JointLimit( **limits )

    @property
    def byJoint(self):
        return self.limits
