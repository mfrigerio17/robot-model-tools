import io, unittest, random

import robmodel.connectivity
from robmodel.connectivity import Link
from robmodel.connectivity import Joint
from robmodel.connectivity import KPair
from robmodel.connectivity import JointKind
from robmodel.convert.postproc import collapseFixedJoints

def _fixj():
    return Joint(f"fjnt{random.random()}", JointKind.fixed)


class CollapseFixedJointsTests(unittest.TestCase):
    def test_single_non_fixed_pair(self):
        l1 = Link("link1")
        l2 = Link("link2")
        j1 = Joint("j1", JointKind.revolute)
        kp = KPair(j1, l1, l2)
        robot = robmodel.connectivity.Robot("robot_test", [kp])

        connectivity = collapseFixedJoints(robot)[0]
        self.assertEqual(connectivity.nB, 2)
        self.assertEqual(connectivity.nJ, 1)

    def test_fixed_leafs(self):
        base = Link("base")
        link1= Link("l1")
        jnt1 = Joint("j1", JointKind.revolute)
        kp1 = KPair(jnt1, base, link1)
        kp2 = KPair(_fixj(), link1, Link("dummy1"))
        kp3 = KPair(_fixj(), link1, Link("dummy2"))
        robot = robmodel.connectivity.Robot("robot_test", [kp1, kp2, kp3])

        connectivity = collapseFixedJoints(robot)[0]
        self.assertEqual(connectivity.nB, 2)
        self.assertEqual(connectivity.nJ, 1)
        self.assertEqual(robot.joints[jnt1.name], jnt1)

    def test_fixed_subchain(self):
        base = Link("base")
        link1= Link("l1")
        link2= Link("l2")
        jnt1 = Joint("j1", JointKind.revolute)
        jnt2 = Joint("j2", JointKind.prismatic)
        dummy= Link("in_the_way")
        pairs = [KPair(jnt1, base, link1), KPair(_fixj(), link1, dummy), KPair(jnt2,dummy, link2)]
        robot = robmodel.connectivity.Robot("robot_test", pairs)

        connectivity = collapseFixedJoints(robot)[0]
        self.assertEqual(connectivity.nB, 3)
        self.assertEqual(connectivity.nJ, 2)
        sorted_joints = list(connectivity.joints.values())

        self.assertEqual(sorted_joints[0], jnt1)
        self.assertEqual(sorted_joints[1], jnt2)

    def test_numbering_scheme(self):
        base = Link("base")
        link11= Link("l11")
        link12= Link("l12")
        link21= Link("l21")
        link31= Link("l31")
        link32= Link("l32")
        jnt11 = Joint("j11", JointKind.revolute)
        jnt12 = Joint("j12", JointKind.revolute)
        jnt31 = Joint("j31", JointKind.revolute)
        jnt32 = Joint("j32", JointKind.revolute)
        kpairs = [ KPair(jnt11, base, link11), KPair(jnt12, base, link12),
            KPair(_fixj(), link11, link21),
            KPair(jnt31, link21, link31), KPair(jnt32, link21, link32)]

        model_name = "robot_test"
        robot = robmodel.connectivity.Robot(model_name, kpairs)

        ids = {'robot':model_name, 'nums':{'base':0, 'l11':1, 'l12':5, 'l21':2, 'l31':3,'l32':4}}
        ordered = robmodel.ordering.Robot(robot, ids)

        id_of_collapsed = ordered.linkNum(link11)

        rob2, ord2 = collapseFixedJoints(robot, ordered)[0:2]
        for link in rob2.links.values():
            id = ord2.linkNum(link)

            if link.name not in robot.links: # new link resulting from collapsing
                self.assertEqual(id, id_of_collapsed)
            else:
                original_id = ordered.linkNum(link)
                if original_id > id_of_collapsed:
                    self.assertEqual(id, original_id-1)
                if original_id < id_of_collapsed:
                    self.assertEqual(id, original_id)


if __name__ == '__main__':
    unittest.main()

