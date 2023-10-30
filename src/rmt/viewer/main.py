import os, logging, argparse
import yaml

import kgprim.core as gr
import rmt.rmt as rmt
import rmt.viewer.meshcat as meshcat_viever

logger = logging.getLogger()

def getPoseSpecsModel(filepath):
    ext = os.path.splitext(filepath)[1]
    if ext == '.yaml' :
        istream = open(filepath)
        data    = yaml.safe_load(istream)
        istream.close()
        meshes_poses = gr.motions.PosesSpec.fromDict(data)
    elif ext == '.motdsl' :
        logger.debug("MotionDSL format detected for file " + filepath)
        try:
            import motiondsl.motiondsl as motiondsl
        except ImportError as e:
            logger.error("MotionDSL support not available!, are you perhaps missing textX in your Python environment?"+
                      " The import error was: " + e.msg())
            exit(-1)
        meshes_tr_model = motiondsl.dsl.modelFromFile(filepath)
        meshes_poses    = motiondsl.toPosesSpecification(meshes_tr_model)
    else :
        logger.error("Unknown pose model file extension '{0}' ({1})".format(ext, filepath))
        exit(-1)

    return meshes_poses


def main():
    '''
    A main to start the Meshcat visualizer independently from the main RMT
    program.
    '''

    formatter = logging.Formatter('%(levelname)s : %(message)s')
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.setLevel(logging.WARN)
    logger.addHandler(handler)

    argparser = argparse.ArgumentParser(prog='rm-viewer', description='Robot model viewer based on Meshcat')
    rmt.setRobotArgs(argparser)
    argparser.add_argument('meshes', metavar='meshes', help='YAML file with paths of the mesh files')
    argparser.add_argument('-t', '--mesh-transforms', dest='mesh_transforms',
        metavar='FILE', default=None,
        help='the specs of the relative pose between mesh and link frame')

    args = argparser.parse_args()
    _, _, _, robotGeometryModel, _, robotGeometryParameters = rmt.getmodels(args.robot, args.params)

    istream = open(args.meshes)
    meshes  = yaml.safe_load(istream)
    istream.close()
    meshesPoses = None
    if args.mesh_transforms is not None :
        meshesPoses = getPoseSpecsModel(args.mesh_transforms)

    meshcat_viever.start(robotGeometryModel, meshes, meshesPoses, robotGeometryParameters)


if __name__ == "__main__" :
    main()
