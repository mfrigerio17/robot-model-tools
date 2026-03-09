import unittest
import robmodel.connectivity as rcn
import robmodel.ordering

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

    def test_iteration_on_ordered_models(self):
        l1 = rcn.Link("l1")
        l2 = rcn.Link("l2")
        l3 = rcn.Link("l3")
        j1 = rcn.Joint("j1", rcn.JointKind.revolute)
        j2 = rcn.Joint("j2", rcn.JointKind.revolute)
        robot = rcn.Robot("robot_test", [rcn.KPair(j1, l1, l2), rcn.KPair(j2, l2, l3)])

        numbering1 = {"robot":robot.name, "nums": {"l1":0,"l2":1,"l3":2}}
        numbering2 = {"robot":robot.name, "nums": {"l1":2,"l2":1,"l3":0}}
        ordered1 = robmodel.ordering.Robot(robot, numbering1)
        ordered2 = robmodel.ordering.Robot(robot, numbering2)

        self.assertEqual(["l1","l2","l3"], [name for name in ordered1.links])
        self.assertEqual(["l3","l2","l1"], [name for name in ordered2.links])

        self.assertEqual(["j1","j2"], [name for name in ordered1.joints])
        self.assertEqual(["j2","j1"], [name for name in ordered2.joints])


if __name__ == '__main__':
    unittest.main()
