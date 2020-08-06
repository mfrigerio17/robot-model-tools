'''
Core types for the connectivity model of an articulated mechanism.
'''

import logging, enum
import networkx as nx
import io

import kgprim.core as gr

class Joint:
    '''
    A placeholder for a joint of a mechanism.

    A joint only has a name and a kind, the latter being one of the values
    defined in `JointKind`.

    A joint always connects two and only two `Link`s.
    '''

    def __init__(self, name, kind):
        self.name = name
        self.kind = kind

    def __eq__(self, rhs):
        return isinstance(rhs, Joint) and\
               self.name==rhs.name and self.kind==rhs.kind
    def __hash__(self):
        return 31 * hash(self.name) + 97 * hash(self.kind)
    def __str__(self):
        return self.name
    def __repr__(self):
        return self.name

class JointKind(enum.Enum):
    prismatic="prismatic",
    revolute ="revolute"

class Link(gr.RigidBody):
    '''
    A placeholder for a rigid link of a mechanism.

    A `Link` is just a named instance of `kgprim.gr.RigidBody`.
    '''
    def __init__(self, name):
        super().__init__(name)

    def __eq__(self, rhs):
        return isinstance(rhs, Link) and self.name==rhs.name
    def __hash__(self):
        return 47 * hash(self.name)
    def __str__(self):
        return self.name

class KPair:
    '''
    A Kinematic-Pair, that is a pair of links connected by a joint
    '''

    def __init__(self, joint, link1, link2):
        self.joint = joint
        self.link1 = link1
        self.link2 = link2

class Robot:
    '''
    The connectivity model of an articulated robot.

    '''

    def __init__(self, name, links, joints, pairs):
        self.log = logging.getLogger('robot')
        self._name = name
        self.links = links  # by-name map
        self.joints= joints # by-name map

        # A map from joint to links in the pair
        self.pairs = {kp.joint: (kp.link1, kp.link2) for kp in pairs}

        self.nB = len(self.links)             # number of bodies
        self.nJ = len(self.joints)            # number of joints
        self.nLoopJ = self.nJ - (self.nB - 1) # number of loop joints

        # The connectivity graph.
        # Note that it can be used as a map bewteen link-pairs to joint
        self.graph = nx.Graph()
        self.graph.add_edges_from( [ (p.link1, p.link2, {'joint': p.joint}) for p in pairs ] )
        self.loops = nx.cycle_basis(self.graph)

        self._checks()

    @property
    def name(self) : return self._name

    def hasLoops(self):
        return self.nLoopJ > 0

    def linkPairToJoint(self, link1, link2):
        '''
        The `Joint` connecting the two given links, `None` if the links are not
        part of a pair.
        '''
        if not self.graph.has_edge(link1, link2) : return None
        return self.graph[link1][link2]['joint']

    def jointToLinkPair(self, joint):
        '''
        The `KPair` object whose joint is the given joint
        '''
        return self.pairs[joint]

    def path(self, link1, link2):
        return nx.shortest_path(self.graph, link1, link2)

    def __str__(self):
        text = 'Robot ' + self.name + '\n'
        for l in self.links.values():
            text += '  ' + l.name + '\n'
        text += '\n'
        for ed in self.graph.edges(data=True):
            text += ed[2]['joint'].name + ' between ' + ed[0].name + ' and ' + ed[1].name + '\n'
        return text

    def _checks(self):
        if not nx.connected.is_connected( self.graph ) :
            self.log.error('The robot graph is not connected')
        else :
            self.log.debug("OK, the robot graph is connected")


def fromDict(data):
    '''
    Create a connectivity model from input data in a dictionary.

    The dictionary is expected to have the following format:

    - a key 'name' with the robot name
    - a key 'links' which is a list of robot link names
    - a key 'joints' which is a list of dictionaries, each having the entries
      'name' and 'kind' which are both strings
    - a key 'pairs' which is a list of dictionaries, each having three entries:
      'joint', 'link1', 'link2', each having a name as the value

    This format is the same as the YAML connectivity model format used in this,
    project. See the sample models in the repository.
    '''
    rname = data['name']
    links = { name:Link(name) for name in data['links'] }
    joints= { j['name']:Joint(j['name'], JointKind[j['kind']]) for j in data['joints'] }
    pairs = [ KPair( joints[ p['joint'] ],
                     links [ p['link1'] ],
                     links [ p['link2'] ] )
                     for p in data['pairs'] ]
    return Robot(rname, links, joints, pairs)


def graphToString(graph):
    text = ''
    for link in graph.nodes() :
        text += link.name + ' '
    return text



