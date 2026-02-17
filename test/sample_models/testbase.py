'''
Common base classes to inherit from, to create test cases

Every class of this module expects robot models and groundtruth data to be
members of the `robotdata` member of the class itself (not the instance).
Subclasses must initialize such member in their `setUpClass`, with data which
is tailored for the specific test cases (e.g. a specific robot model, imported
from a specific format, etc.)
'''

import unittest, logging, io
import robmodel.convert.urdf.imp as urdfin
import robmodel.convert.urdf.exp as urdfout
from robmodel.treeutils import TreeUtils


class RobotTestBase(unittest.TestCase):
    def setUp(self):
        '''
        Copy a reference of the models expected in the class to the fields of
        this instance. This way the test code can access
        the robot model data and ground-truth data via `self`.
        '''
        cls = self.__class__
        self.robot = cls.robotdata.ordering
        self.connectivity = cls.robotdata.connectivity
        self.frames = cls.robotdata.frames
        self.treeu = cls.robotdata.treeutils
        self.groundtruth  = cls.robotdata.groundtruth


class ConvertToURDFTestBase(RobotTestBase):
    @classmethod
    def setUpClass(cls):
        '''
        Convert the robot models - expected as members of the class - with
        the URDF backend
        '''
        # generate the URDF
        urdf = urdfout.modelText(cls.robotdata.geometry, inertiaModel=cls.robotdata.inertia, includeDummies=False)
        # load the generated model
        urdfwrap = urdfin.URDFWrapper( io.StringIO(urdf) )
        connectivity, ordering, frames, geometry, inertia = urdfin.convert(urdfwrap)[0:5]

        cls.robotdata.robot = ordering
        cls.robotdata.connectivity = connectivity
        cls.robotdata.frames = frames
        cls.robotdata.treeu = TreeUtils(ordering)


from sample_models import logger

def main():
    formatter = logging.Formatter('%(levelname)s (%(name)s) : %(message)s')
    handler   = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.setLevel(logging.INFO)
    logger.addHandler(handler)

    unittest.main()

