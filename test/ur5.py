from robmodel.treeutils import TreeUtils

from testcore import BasicTests, TreeTests
import robottest
from robottest import SampleRobotModel, RobotTestBase

'''
Test classes for the UR5 robot model contained in the `sample/` folder.

Tests are created by inheriting and mixing the base classes defined in the
other modules, and the ur5-specific classes in this module.
'''

class UR5KinDSLModel(SampleRobotModel):
    def __init__(self, **kwds):
        super().__init__(name='ur5', extension='kindsl', **kwds)

class UR5URDFModel(SampleRobotModel):
    def __init__(self, **kwds):
        super().__init__(name='ur5', extension='urdf', **kwds)

class UR5YAMLModel(SampleRobotModel):
    def __init__(self, **kwds):
        super().__init__(name='ur5', extension='yaml', **kwds)

class UR5TestData:
    '''
    The class that defines the ground-truth data about a robot, such as the
    number of joints, or the parentship relation.

    A class like this one must be manually written for each robot model that one
    wants to test.
    '''

    def __init__(self, **kwds):
        super().__init__(**kwds)
        gt = {}
        gt['parents'] = {
            'base' : None, 'shoulder':'base', 'upperarm':'shoulder',
            'forearm':'upperarm', 'wrist_1':'forearm', 'wrist_2':'wrist_1',
            'wrist_3':'wrist_2'
        }

        gt['leafs'] = ['wrist_3']
        gt['nB'] = 7  # number of links
        gt['nJ'] = 6  # number of joints

        self.groundtruth = gt
        self.treeutils   = TreeUtils(self.ordering)


# mix the ground-truth data with the model data:

class UR5_data_kindsl(UR5TestData, UR5KinDSLModel): pass
class UR5_data_urdf  (UR5TestData, UR5URDFModel  ): pass
class UR5_data_yaml  (UR5TestData, UR5YAMLModel  ): pass

class UR5_tests_kindsl( BasicTests, TreeTests, RobotTestBase):
    @classmethod
    def setUpClass(cls):
        cls.robotdata = UR5_data_kindsl()

class UR5_tests_urdf( BasicTests, TreeTests, RobotTestBase):
    @classmethod
    def setUpClass(cls):
        cls.robotdata = UR5_data_urdf()

class UR5_tests_yaml( BasicTests, TreeTests, RobotTestBase):
    @classmethod
    def setUpClass(cls):
        cls.robotdata = UR5_data_yaml()



if __name__ == '__main__':
    robottest.main()
