import os, logging
import unittest

import rmt.rmt

this_dir   = os.path.dirname(__file__)
models_dir = os.path.join( os.path.join(this_dir, os.pardir), 'sample', 'models') # i.e.: cd ../sample/models

logger = logging.getLogger(None) # get the "root" logger

class SampleRobotModel:
    '''
    Loader of models contained in the sample/ folder
    '''

    def __init__(self, name, extension):
        rpath = os.path.join( name, name+'.'+extension ) # <name>/<name>.<extension>
        rfile = os.path.join( models_dir, rpath )
        logger.debug("Trying to load {0}".format(rfile))
        self.connectivity, self.ordering, self.frames, self.geometry, self.inertia = rmt.rmt.getmodels(rfile)


class RobotTestBase(unittest.TestCase):
    def setUp(self):
        '''
        Expect robot-data to be in the class, and copy a reference to the
        fields in the instance (i.e. self), so that test code can use the robot
        data
        '''
        cls = self.__class__
        self.robot = cls.robotdata.ordering
        self.connectivity = cls.robotdata.connectivity
        self.frames = cls.robotdata.frames
        self.treeu = cls.robotdata.treeutils
        self.groundtruth  = cls.robotdata.groundtruth

def main():
    logger = logging.getLogger(None) # get the "root" logger
    formatter = logging.Formatter('%(levelname)s (%(name)s) : %(message)s')
    handler   = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.setLevel(logging.INFO)
    logger.addHandler(handler)
    unittest.main()