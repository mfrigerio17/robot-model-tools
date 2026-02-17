import pathlib
import rmt.rmt
from robmodel.treeutils import TreeUtils

from sample_models import logger

this_dir   = pathlib.Path(__file__).parent
models_dir = this_dir.parent.parent / 'sample' / 'models'

class SampleRobotModel:
    '''
    Utility base class to load models contained in the `sample/` folder
    '''

    def __init__(self, name, extension):
        '''
        - `name`: the name of the robot model to load
        - `extension`: the model file extension, e.g. "kindsl", "urdf", ...

        This function will try to load the model `sample/<name>/<name>.<extension>`
        '''
        rpath = pathlib.Path(name) / (name+'.'+extension) # <name>/<name>.<extension>
        rfile = models_dir / rpath
        logger.debug("Trying to load {0}".format(rfile))
        self.connectivity, self.ordering, self.frames, self.geometry, self.inertia = rmt.rmt.getmodels(str(rfile))[0:5]
        self.treeutils = TreeUtils(self.ordering)
