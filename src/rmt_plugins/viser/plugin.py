import time, pathlib
import rmt.rmt as core
import rmt.load as loadutils

import rmt_plugins.viser.plugin as thisplugin
import rmt_plugins.viser.viser as thisviser
from rmt_plugins.viser import logger

def customize_cmdline_args(core_argparser, core_subparsers_group):
    argparser = core_subparsers_group.add_parser('viser', parents=[core_argparser], help='Launch a robot model viewer based on Viser')
    argparser.add_argument('-m', '--mesh-paths', dest='meshes', help='dictionary file with paths of the mesh files (YAML/JSON)')
    argparser.set_defaults(func= thisplugin.launch)

def customize_options(parsed_arguments, opts_dict):
    #opts_dict["meshes"] = parsed_arguments.meshes
    pass

# --------------------------------------------------------------------------- #

def launch(args, opts):
    robotGeometryModel = core.getmodels(args.robot, **opts)[3]

    meshesPaths = {}
    if args.meshes:
        indict = loadutils.loadDictionary(args.meshes)
        if "model" not in indict:
            logger.warning("Missing 'model' key in the dictionary in '%s'", args.meshes)
        else:
            if indict["model"] != robotGeometryModel.robotName :
                logger.warning("Mismatch between the robot name and the name in the mesh paths file")
            del indict["model"]
        meshesPaths = { name:pathlib.Path(indict[name]) for name in indict.keys() }

    server = thisviser.makeServer()
    scene = thisviser.ViserScene(robotGeometryModel, server, meshesPaths)
    scene.loadRobotIntoScene()

    while True:
        time.sleep(10.0)
