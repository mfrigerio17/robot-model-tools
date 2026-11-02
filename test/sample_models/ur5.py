import sample_models.testcore
import sample_models.loader
import sample_models.testbase
from sample_models.testcore import BasicTests, TreeTests
from sample_models.loader import SampleRobotModel
from sample_models.testbase import RobotTestBase

'''
Test classes for the UR5 robot model contained in the `sample/` folder.

Tests are created by inheriting and mixing the base classes defined in the
other modules, and the ur5-specific classes in this module.
'''

class UR5TestData:
    '''
    The class that defines the ground-truth data about the UR5 model, such as the
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


# mix the ground-truth data with the model data:

class _UR5_data(UR5TestData, SampleRobotModel):
    def __init__(self, model_file_extension, **kwds):
        super().__init__(name='ur5', extension=model_file_extension, **kwds)



class UR5_tests_kindsl( BasicTests, TreeTests, RobotTestBase):
    @classmethod
    def setUpClass(cls):
        cls.robotdata = _UR5_data(model_file_extension='kindsl')

class UR5_tests_urdf( BasicTests, TreeTests, RobotTestBase):
    @classmethod
    def setUpClass(cls):
        cls.robotdata = _UR5_data(model_file_extension='urdf')

class UR5_tests_yaml( BasicTests, TreeTests, RobotTestBase):
    @classmethod
    def setUpClass(cls):
        cls.robotdata = _UR5_data(model_file_extension='yaml')


if __name__ == '__main__':
    sample_models.testbase.main()
