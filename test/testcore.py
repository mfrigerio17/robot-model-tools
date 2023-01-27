import logging

logger = logging.getLogger(None) # get the "root" logger

'''
Common, basic tests that can be performed on loaded robot models.

The "self" instance is expected to have members for the loaded models, as well
as ground-truth data for comparisons. See the `ur5.py` module for an example
of how to setup actual tests for an actual model.
'''

class BasicTests:
    def test_counts(self):
        self.assertTrue( self.connectivity.nB == self.groundtruth['nB'] )
        self.assertTrue( self.connectivity.nJ == self.groundtruth['nJ'] )
        self.assertTrue( self.connectivity.nB == len(self.connectivity.links) )
        self.assertTrue( self.connectivity.nJ == len(self.connectivity.joints) )

    def test_frame_counts(self):
        self.assertTrue( len(self.frames.linkFrames)==len(self.connectivity.links) )
        self.assertTrue( len(self.frames.jointFrames)==len(self.connectivity.joints) )

    def test_numeric_ids(self):
        '''The ID of a joint must be the same as the ID of its successor link'''
        for joint in self.connectivity.joints.values() :
            jid = self.robot.jointNum(joint)
            successor = self.robot.successor(joint)
            sid = self.robot.linkNum(successor)
            self.assertTrue( jid == sid )


class TreeTests:
    def test_parents(self):
        parents = self.groundtruth['parents']
        for link in self.robot.links.values() :
            if parents[link.name] == None :
                self.assertTrue( self.treeu.parent(link)==None )
            else :
                real = parents[link.name]
                candidate = self.treeu.parent(link).name
                same = (real == candidate)
                if not same :
                    logger.error("For link '{l}', expected parent: '{r}', got: '{c}'".format(l=link.name, r=real, c=candidate))
                self.assertTrue( same )

    def test_leafs(self):
        for leaf in self.groundtruth['leafs'] :
            link = self.robot.links[ leaf ]
            self.assertTrue( self.treeu.isLeaf(link) )

    def test_ancestors(self):
        for link in self.robot.links.values() :
            self.assertTrue( self.treeu.ancestorOf( self.robot.base, link) )


