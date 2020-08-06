import networkx as nx

class TreeUtils:
    '''
    Helper object for the inspection of the tree structure of a robot model.

    This class works only with an ordered connectivity model with tree topology.
    An ordered connectivity model is an instance of `robmodel.ordering.Robot`.

    With an ordered kinematic tree (i.e. a model with a numbering scheme for the
    links) additional parentship relations can be established among the robot
    links. This class exposes such relations.
    '''

    def __init__(self, rob):
        '''
        The `rob` argument must be a kinematic tree model with a numbering
        scheme.
        '''
        if not hasattr(rob, 'dgraph') :
            raise RuntimeError('''Tree-utils can only be constructed with
            an ordered robot model, i.e. one whose treeModel graph is
            directed''')
        if rob.hasLoops() :
            raise RuntimeError("Kinematic loops detected, TreeUtils only accepts kinematic trees")

        self.robot = rob
        self.parentToChild = rob.dgraph
        self.childToParent = nx.reverse_view(self.parentToChild)

    def connectingJoint(self, l1, l2):
        '''
        The joint joining the two given links, None if there is no such a joint.
        '''
        return self.robot.linkPairToJoint(l1, l2)

    def parent(self, l):
        '''
        The parent link of the given link
        '''
        if l == self.robot.base : return None
        return self.childToParent.neighbors(l).__next__()

    def children(self, l):
        '''
        The list of children of the given link
        '''
        return self.parentToChild.neighbors(l)

    def isLeaf(self, l):
        '''
        Return True if the given link has no children
        '''
        return len(self.parentToChild._succ[l]) == 0

    def supportingJoint(self, l):
        '''
        The joint for which the given link is the successor.

        None if the link is the robot base.
        '''
        if l == self.robot.base : return None
        return self.connectingJoint(l, self.parent(l))

    def ancestorOf(self, possibleAncestor, target):
        '''
        Tells whether a link is an ancestor of another link.
        Any link is an ancestor/descendant of itself.

        Returns True if `target` belongs to a kinematic subtree rooted at
        `possibleAncestor`, False otherwise.
        '''

        if ((possibleAncestor == None) or (target == None)):
            return False
        if self.robot.linkNum(possibleAncestor) > self.robot.linkNum(target) :
            return False
        return possibleAncestor==target or self.ancestorOf(possibleAncestor, self.parent(target))


    def lowestCommonAncestor(self, l1, l2):
        '''
        The lowest common ancestor of the two given links.

        The lowest common ancestor is the deepest (i.e. farthest from the base)
        link which is an ancestor of both the arguments.
        '''

        if ((l1 == None) or (l2 == None)):
            return None
        lca = None
        current1 = l1
        current2 = l2
        while lca == None :
            if self.ancestorOf(current1, l2):
                lca = current1
            else:
                current1 = self.parent(current1) #// cant be null, as that implies current1 is the base, and the base is always an ancestor
                if self.ancestorOf(current2, l1):
                    lca = current2
                else:
                    current2 = self.parent(current2)
        return lca



