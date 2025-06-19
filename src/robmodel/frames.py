'''
Model of the set of Cartesian frames attached to an articulated robot.

At the moment, this module provides only `RobotDefaultFrames`, which represents
a specific convention about the attachment of frames:

- every `Link` of the robot has one and only one frame used as a reference
- every `Joint` of the robot also has an associated frame
- every `Link` may have any number of additional custom frames, which shall be
  located with respect to the main reference frame.
'''

import enum
import kgprim.core as primitives


class FrameRole(enum.Enum):
    '''
    Possible roles a reference frame attached to a link might have.

    A frame can either be (a) the main, default reference for the link, (b) the
    one locating a joint for which the link is the predecessor, (c) some other
    application-specific role.
    '''

    linkRef= 'linkRef'
    joint  = 'joint'
    user   = 'user'


class FrameRelationKind(enum.Enum):
    '''
    The possible relations between proximal neighbor frames.

    Proximal neighbor frames are at distance 1 from each other.
    Distance 1 means that the relative pose between the two frames is a model
    constant/parameter (and not a composition of multiple relative poses).

    The relations between distance-1 frames are: between a link-frame and a
    custom user-defined frame, between a joint-frame and the predecessor-frame,
    between the joint-frame and the successor-frame. Note that the first two
    correspond to constant poses, the last one to a pose dependent on the joint
    status.
    '''
    generic    = 'generic'
    linkJoint  = 'linkJoint'
    acrossJoint= 'acrossJoint'


def linkFrameName(robot, link):
    return link.name

def jointFrameName(robot, joint):
    if joint.name in robot.links.keys():
        # if the joint has the same name as a link, we cannot use that name
        # for the frame name, because there will be a duplicate
        return "j_" + joint.name
    else:
        return joint.name



import networkx as nx

# TODO read external specs with attached frames, and check for various
# constraints:
# - each link must have one and only one frame that does not depend on others,
#   pure symbolic (the default link frame)
# - all the other frames attached to the link must be speficied wrt to the
#   default frame, or another frame already defined
# - each link must have a frame for each joint
# - frames on different links, which however refer to the same joint, must be
#   recognizable. These are the frames that coincide at the zero joint status



class RobotDefaultFrames():

    def path(self, frame1, frame2):
        return nx.shortest_path(self.graph, frame1, frame2)


    def isIdentity(self, frame1, frame2):
        return self._edge(frame1, frame2)['identity']

    def kind(self, frame1, frame2):
        return self._edge(frame1, frame2)['kind']

    def joint(self, frame1, frame2=None):
        if frame2 is None:
            if self.frameRole(frame1) == FrameRole.joint :
                return self.graph.nodes[frame1]['joint']
        else :
            ed = self._edge(frame1, frame2)
            if ed['kind'] is FrameRelationKind.acrossJoint :
                return ed['joint']
        return None

    def frameRole(self, frame):
        '''
        The `FrameRole` attribute of the given frame.

        The argument should be a frame retrieved from this instance.
        '''
        return frame.attrs['role']

    def poseRelativeToSupportingLinkFrame(self, frame):
        relativeTo = self.linkFrames[ frame.body ]
        return primitives.Pose(target=frame, reference=relativeTo)

    def __init__(self, robot, userAttachedFrames):
        '''
        Arguments:

        - `robot`: a `robmodel.ordering.Robot` instance, that is, a connectivity
          model with an associated numbering
        '''
        if not hasattr(robot, 'dgraph') :
            raise RuntimeError('''The default-framesModel model can only be
            constructed with an ordered robot model, i.e. one whose
            treeModel graph is directed''')
        self.robot = robot
        self.linkFrames  = {}
        self.jointFrames = {}
        self.framesByName= {}
        self.userAttachedFrames = userAttachedFrames
        self._makeGraph()

    def __iter__(self):
        return iter(self.framesByName)

    def getAttachedFrame(self, frame):
        for atfr in self.graph.nodes():
            if atfr.name == frame.name : return atfr
        return None


    def _edge(self, frame1, frame2):
        if not self.graph.has_edge(frame1, frame2) :
            raise ValueError("The two given frames are not neighbors")
        return self.graph[frame1][frame2]
    #
    # Constructs a graph whose vertices are all the frames attached
    # to the given robot. Follows robcogen convention. Assumes ordering
    # between link1 and link2 of each pair.
    #
    def _makeGraph(self):
        graph = self.graph = nx.Graph()

        for link in self.robot.links.values() :
            frame = self._linkFrame(link)
            graph.add_node( frame )

            self.linkFrames[link] = frame
            self.framesByName[frame.name] = frame

        for joint in self.robot.joints.values() :
            pre = self.robot.predecessor(joint)
            lf1 = self.linkFrames[ pre ]
            lf2 = self.linkFrames[ self.robot.successor(joint) ]
            jf1 = self._linkJointFrame(pre, joint)
            graph.add_node( jf1, joint=joint )
            graph.add_edge(lf1, jf1, kind=FrameRelationKind.linkJoint, identity=False)
            graph.add_edge( jf1, lf2,
                            kind=FrameRelationKind.acrossJoint,
                            joint=joint,
                            identity=False)
            self.framesByName[jf1.name] = jf1
            self.jointFrames[ joint ] = jf1

        for userFrame in self.userAttachedFrames :
            userFrame.attrs['role'] = FrameRole.user
            graph.add_node( userFrame )
            link = userFrame.body  # a property of primitives.Attachment
            graph.add_edge( self.linkFrames[link], userFrame, kind=FrameRelationKind.generic )
            self.framesByName[ userFrame.name ] = userFrame

        self.graph = graph
        return graph

    def _linkFrame(self, link):
        fName = linkFrameName(self.robot, link)
        aframe = primitives.Attachment( primitives.Frame(fName), link )
        aframe.attrs['role'] = FrameRole.linkRef
        return aframe

    def _linkJointFrame(self, link, joint):
        fName = jointFrameName(self.robot, joint)
        aframe = primitives.Attachment( primitives.Frame(fName), link )
        aframe.attrs['role'] = FrameRole.joint
        return aframe

    @property
    def byLink(self):
        '''A dictionary mapping a Link to its attached Frame'''
        return self.linkFrames

    @property
    def byJoint(self):
        '''A dictionary mapping a Joint to its attached Frame'''
        return self.jointFrames

    @property
    def byName(self):
        '''A dictionary mapping the Frame name to the actual Frame'''
        return self.framesByName
