import unittest
import robmodel.connectivity as rcn

class ConnectivityTests(unittest.TestCase):
    def _make_malformed(self):
        l1 = rcn.Link("link1")
        rcn.Robot("robot_malformed", None)

    def test_malformed(self):
        self.assertRaises(Exception, self._make_malformed)

    def test_single_kinematic_pair(self):
        l1 = rcn.Link("link1")
        l2 = rcn.Link("link2")
        j1 = rcn.Joint("j1", rcn.JointKind.revolute)
        kp = rcn.KPair(j1, l1, l2)
        robot = rcn.Robot("robot_test", [kp])

        self.assertTrue(robot.nB == 2)
        self.assertTrue(robot.nJ == 1)
        self.assertTrue(robot.nLoopJ == 0)
        self.assertEqual( robot.linkPairToJoint(l1,l2), j1 )
        self.assertEqual( robot.linkPairToJoint(l2,l1), j1 )
        self.assertEqual( robot.jointToLinkPair(j1), (l1,l2) )



if __name__ == '__main__':
    unittest.main()
