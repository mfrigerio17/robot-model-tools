'''
Numbering scheme to be composed with a connectivity model.

A numbering scheme introduces an ordering in the robot links, and the
"predecessor"/"successor" relation between joints and links: for any kinematic
pair, the predecessor of the joint is the link with the lower number, the
successor is the other one. The numbering also defines which link of the robot
is the "base", whose code is 0.

See `robmodel.connectivity`.
'''

import logging
import networkx as nx
import robmodel.connectivity

log = logging.getLogger(__name__)

class Robot(robmodel.connectivity.Robot):
    '''
    The composition of a basic robot model with a numbering scheme for the links.

    A numeric code for each link and joint is required.
    The codes for the links must be integers from 0 to NB-1, in sequence, where
    NB is the number of links.

    Tree joints do not need an explicit code, because they get the same code as
    the successor link.
    Loop joints, on the other hand, do need to be given a code by the user.
    Joint codes range from 1 to NJ, where NJ is the number of joints.

    In conclusion, a code for each link and each loop joint is required.
    This amounts to a number of codes equal to the total number
    of joints plus 1 (for the robot base link).
    '''

    def __init__(self, robotm, numbering):
        '''
        Arguments:

        - `robotm`: a connectivity model, typically an instance of
          `robmodel.connectivity.Robot`
        - `numbering`: a dictionary with two keys, "robot" and "nums". The value
           of "robot" must be the same as the connectivity model name. The value
           of "nums" must be another dictionary with link names and loop joint
           names as keys, and numeric codes as values.
        '''
        if 'robot' not in numbering :
            raise RuntimeError("Expected key 'robot' in the numbering-scheme dictionary")
        if robotm.name != numbering['robot'] :
            raise ValueError('''The given numbering scheme and robot model do
                             not match ({0} vs {1})'''
                             .format(numbering.robot, robotm.name))

        nums = numbering['nums']

        if not self._consistency(robotm, nums) :
            raise RuntimeError("Inconsistent arguments")

        self.connectivity = robotm

        self.itemNameToCode = nums
        self.codeToLink = {nums[l.name] : l for l in robotm.links.values() }

        self.codeToJoint = {}
        self.jointToCode = {}
        for joint in robotm.joints.values() :
            # If there is an entry for a joint, it means that was selected as
            # a loop joint
            if joint.name in nums:
                self.jointToCode[ joint ] = nums[joint.name]
                self.codeToJoint[ nums[joint.name] ] = joint
            else :
                # Otherwise it is a tree joint, and it gets the highest code
                # among the codes of the two links of its pair
                code = max( [nums[l.name] for l in robotm.jointToLinkPair(joint)] )
                self.jointToCode[ joint ] = code
                self.codeToJoint[ code ] = joint

        dGraph = nx.DiGraph()
        dGraph.add_nodes_from( robotm.graph ) # links
        # Add directed edges starting from the link with the lower ID
        self.orderedPairs = {}
        for joint in robotm.joints.values() :
            kp = robotm.jointToLinkPair( joint )
            if nums[kp[0].name]<nums[kp[1].name]:
                dGraph.add_edge( kp[0], kp[1], joint=joint)
                self.orderedPairs[joint] = (kp[0], kp[1])
            else:
                dGraph.add_edge( kp[1], kp[0], joint=joint)
                self.orderedPairs[joint] = (kp[1], kp[0])

        self.dgraph = dGraph

    @property
    def base(self):  return self.codeToLink[0]

    def linkNum (self, link): return self.itemNameToCode[link.name]

    def jointNum(self, joint): return self.jointToCode[joint]

    def predecessor(self, joint): return self.orderedPairs[joint][0]

    def successor(self, joint): return self.orderedPairs[joint][1]

    def __getattr__(self, name):
        return getattr(self.connectivity, name)

    def _consistency(self, connectivity, nums):
        links = connectivity.links

        if len(nums) != (connectivity.nJ + 1) :
            log.error("Wrong number of entries in the numbering scheme (found {0}, expected {1})"
                      .format(len(nums), connectivity.nJ+1) )
            return False

        for link_name in links.keys() :
            if link_name not in nums :
                log.error("Link '{l}' not found in the numbering scheme".format(l=link_name))
                return False
        for link_name in nums.keys() :
            if link_name not in links :
                log.warning("Name '{l}' (referenced in the numbering scheme) is not a link of robot '{r}'"
                            .format(l=link_name, r=robotm.name))

        # Check for duplicate or missing IDs
        max = len(links)-1
        ids = list( nums.values() )
        ids.sort()
        prev = -22
        for id in ids :
            if id == prev : log.warning("Duplicate link ID {0}".format(id))
            if id > max :   log.warning("Link ID {0} out of range".format(id))
            prev= id
        for i in range(0, max+1) :
            if i not in ids : log.warning("Missing link ID {0}".format(i))

        return True


    def __str__(self):
        text = 'Robot ' + self.name + '\n'
        for code in range(0,len(self.links)) :
            text += "Link {0} (# {1})\n".format(self.codeToLink[code].name, code)
        text += '\n'
        for joint in self.joints.values() :
            lp = self.jointToLinkPair(joint)
            text += "Joint {0} (# {1}) between {2} and {3}\n".format(
                joint.name, self.jointNum(joint), lp[0].name, lp[1].name)
        return text



def numberingSchemeFromDict(data):
    rname = data['robot']
    nums  = data['nums']
    num = {'robot' : rname, 'nums': nums}
    return num






