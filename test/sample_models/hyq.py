import sample_models.testcore
import sample_models.loader
import sample_models.testbase
from sample_models.testcore import BasicTests, TreeTests
from sample_models.loader import SampleRobotModel
from sample_models.testbase import RobotTestBase, ConvertToURDFTestBase


'''
Test classes for the HyQ robot model contained in the `sample/` folder.

Tests are created by inheriting and mixing the base classes defined in the
other modules, and the HyQ-specific classes in this module.
'''

class ModelGroundTruthData:
    def __init__(self, **kwds):
        super().__init__(**kwds) # do not delete, required for multiple inheritance/composition
        gt = {}
        gt['parents'] = {
            'trunk' : None,
            'LF_hipassembly':'trunk',
            'RF_hipassembly':'trunk',
            'LH_hipassembly':'trunk',
            'RH_hipassembly':'trunk',
            'LF_upperleg' : 'LF_hipassembly',
            'RF_upperleg' : 'RF_hipassembly',
            'LH_upperleg' : 'LH_hipassembly',
            'RH_upperleg' : 'RH_hipassembly',
            'LF_lowerleg' : 'LF_upperleg',
            'RF_lowerleg' : 'RF_upperleg',
            'LH_lowerleg' : 'LH_upperleg',
            'RH_lowerleg' : 'RH_upperleg'
        }

        gt['leafs'] = ['LF_lowerleg','RF_lowerleg','LH_lowerleg','RH_lowerleg']
        gt['nB'] = 13  # number of links
        gt['nJ'] = 12 # number of joints

        self.groundtruth = gt

# Compose the loaded robot models with handwritten ground-truth data
class HyQ_data_kindsl(ModelGroundTruthData, SampleRobotModel):
    def __init__(self, **kwds):
        super().__init__(name='hyq', extension='kindsl', **kwds)


class HyQ_tests_kindsl(RobotTestBase, BasicTests, TreeTests):
    @classmethod
    def setUpClass(cls):
        cls.robotdata = HyQ_data_kindsl()
        super().setUpClass()


class HyQ_tests_kindsl2urdf(ConvertToURDFTestBase, BasicTests, TreeTests):
    '''
    Run the common tests, but on models which are obtained via a URDF conversion
    of the original model. See `ConvertToURDFTestBase`.
    '''
    @classmethod
    def setUpClass(cls):
        cls.robotdata = HyQ_data_kindsl()
        super().setUpClass()


if __name__ == '__main__':
    sample_models.testbase.main()
