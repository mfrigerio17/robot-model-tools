import time, pathlib, logging, argparse
import rmt.rmt as core
import rmt.load as loadutils

import rmt.viewer.viser as myview

logger = logging.getLogger(__name__)

def main():
    formatter = logging.Formatter('%(levelname)s (%(name)s) : %(message)s')
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger = logging.getLogger() # get the root one, to change settings across modules
    logger.setLevel(logging.WARN)
    logger.addHandler(handler)

    core.loadPlugins()
    argparser = argparse.ArgumentParser(prog='rm-viewer', description='Robot model viewer based on Viser')
    core.setRobotArgs(argparser)
    argparser.add_argument('-m', '--mesh-paths', dest='meshes', help='dictionary file with paths of the mesh files (YAML/JSON)')
    args = argparser.parse_args()
    opts = core.optsDict(args)

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

    server = myview.makeServer()
    scene = myview.ViserScene(robotGeometryModel, server, meshesPaths)
    scene.loadRobotIntoScene()

    while True:
        time.sleep(10.0)


if __name__ == "__main__":
    main()
