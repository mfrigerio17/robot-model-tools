import os, sys, logging, argparse
import networkx as nx
import numpy as np

import robmodel.convert.urdf.imp as urdfin

import robmodel.inertia
import robmodel.jposes

import rmt.kinematics

log = logging.getLogger() # get the root logger, this is a command line app


def getmodels(filepath, paramsfilepath=None, floatLiteralsAsConstants=False):
    connectivity = None
    ordering     = None
    frames       = None
    geometry     = None
    inertia      = None
    _, ext = os.path.splitext(filepath)
    if ext == '.urdf' :
        log.debug("URDF format detected for file " + filepath)
        try:
            urdffile = open(filepath)
            urdfwrap = urdfin.URDFWrapper(urdffile)
            connectivity, ordering, frames, geometry = urdfin.convert(urdfwrap)
        except Exception as e:
            log.error("Failed to load URDF model")
        finally:
            urdffile.close()

    elif ext == '.kindsl' :
        log.debug("KinDSL format detected for file " + filepath)
        try:
            import robmodel.convert.kindsl.imp as kindslin
        except ImportError as e:
            log.error("KinDSL support not available!, are you perhaps missing textX in your Python environment?"+
                      " The import error was: " + e.msg())
            exit(-1)
        params = {}
        if paramsfilepath is not None:
            istream = open(paramsfilepath)
            params  = yaml.load(istream)
            istream.close()
        connectivity, ordering, frames, geometry, inertia = kindslin.convert(filepath, params)

    elif ext == '.yaml' :
        import yaml
        import robmodel.convert.yaml.imp as yamlin
        log.debug("YAML format detected for file " + filepath)
        istream = open(filepath)
        data = yaml.safe_load(istream);
        istream.close()
        basepath = os.path.dirname(filepath)
        if 'connectivity' not in data:
            raise RuntimeError('Could not find the path of the connectivity model in {0}'.format(filepath))
        path = os.path.join(basepath, data['connectivity'])
        istream = open(path)
        connectivity = yamlin.connectivity( istream )
        istream.close()
        if 'numbering' in data:
            path = os.path.join(basepath, data['numbering'])
            istream = open(path)
            nscheme = yamlin.numbering_scheme(istream)
            istream.close()
            ordering = robmodel.ordering.Robot( connectivity, nscheme )
            frames   = robmodel.frames.RobotDefaultFrames(ordering, [])
            if 'geometry' in data:
                path = os.path.join(basepath, data['geometry'])
                istream = open(path)
                metric = yamlin.geometry(istream, floatLiteralsAsConstants)
                istream.close()
                geometry = robmodel.geometry.Geometry(ordering, frames, metric)
            if 'inertia' in data :
                filepath = os.path.join(basepath, data['inertia'])
                istream = open(filepath)
                inertia_data = yamlin.inertia(istream, floatLiteralsAsConstants);
                istream.close()
                if inertia_data == None :
                    raise RuntimeError("Could not load inertia data")
                inertia = robmodel.inertia.RobotLinksInertia(connectivity, frames, inertia_data)

    else :
        log.error("Unknown robot model extension '{0}'".format(ext))
        exit(-1)

    return connectivity, ordering, frames, geometry, inertia

def defpose(args):
    robot,frames,geometry = getmodels(args.robot, args.params)[1:4]
    jointPoses = robmodel.jposes.JointPoses(robot, frames)
    kin = rmt.kinematics.RobotKinematics(geometry, jointPoses)
    H = rmt.kinematics.base_H_ee(kin, args.frame)
    if H is None :
        log.error("Could not compute the frame pose")
        exit(-1)
    print(np.round(H,5))

def printinfo(args):
    c,o,f,g,i = getmodels(args.robot)
    print(c,o,f,g,i)

def writeDOTFile(args):
    connectivity = getmodels(args.robot)[0]
    # Convert the graph to AGraph format used by pygraphviz
    ag = nx.nx_agraph.to_agraph( connectivity.graph )
    # Add the edge labels (joint names), to have them displayed
    for e in ag.edges() :
        e.attr['label'] = e.attr['joint']
    ag.write(args.outdot)
    return ag

def writeMotDSLFile(args):
    ostream = sys.stdout
    closeOstream = False
    if args.outmotdsl is not None :
        try:
            ostream = open(args.outmotdsl, 'w')
            closeOstream = True
        except IOError :
            log.warning("Could not open file '{0}'".format(args.outmotdsl))

    robot,frames,geometry = getmodels(args.robot, args.params)[1:4]
    jointPoses = robmodel.jposes.JointPoses(robot, frames)
    robotKin   = rmt.kinematics.RobotKinematics(geometry, jointPoses)
    rmt.kinematics.serializeToMotionDSLModel(robotKin, ostream)

    if closeOstream :
        ostream.close()

def export(args):
    c,o,f,g,i = getmodels(args.robot)
    oformat = args.oformat
    if oformat is None: oformat = 'yaml'

    if oformat == 'yaml' :
        log.error('Sorry, not implemented yet')
        exit(-1)
    elif oformat == 'kindsl' :
        try:
            import robmodel.convert.kindsl.exp as kindslout
        except ImportError as e:
            log.error("KinDSL support not available!, are you perhaps missing textX in your Python environment?"+
                    " The import error was: " + e.msg())
            exit(-1)
        if g is not None:
            text = kindslout.geometry(g)
        else :
            log.error("Sorry, I can export to KinDSL only a complete geometry model ")
            exit(-1)
    elif oformat == 'urdf' :
        import robmodel.convert.urdf.exp as urdfout
        if g is not None :
            text = urdfout.geometry(g)
        elif o is None:
            log.error("Cannot export a URDF is the input model that does not even include ordering")
            exit(-1)
        else :
            text = urdfout.ordering(o)
    else :
        log.error("Unknown robot model format '{0}'".format(ext))
        exit(-1)

    ostream = sys.stdout
    closeOstream = False
    outfile = args.outfile
    if outfile is not None :
        try:
            ostream = open(outfile, 'w')
            closeOstream = True
        except IOError :
            log.error("Could not open file '{0}'".format(outfile))
            exit(-1)
    ostream.write(text)
    if closeOstream :
        ostream.close()

def playground(args):
    inertia = getmodels(args.robot)[4]
    for link in inertia.inertia :
        print(inertia.inertia[link])


def setRobotArgs(argparser):
    argparser.add_argument('robot', metavar='robot-model', help='the robot model input file')
    argparser.add_argument('-p', '--params', dest='params', metavar='params-file', default=None, help='YAML file with default parameter values')

def main():
    formatter = logging.Formatter('%(levelname)s (%(name)s) : %(message)s')
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    log.setLevel(logging.WARN)
    log.addHandler(handler)

    argparser = argparse.ArgumentParser(prog='rmt', description='Performs misceallaneous operations on robot models')
    argparser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help='lower the logging level to DEBUG')
    subparsers= argparser.add_subparsers()

    parser = subparsers.add_parser('defpose', help="Print the pose of a frame relative to the base frame, assuming the zero-configuration of the robot")
    setRobotArgs(parser)
    parser.add_argument('frame', metavar='frame', help='the name of the frame whose pose is of interest')
    parser.set_defaults(func=defpose)

    parser = subparsers.add_parser('print', help='Dump to stdout some serialization of the robot model components')
    setRobotArgs(parser)
    parser.set_defaults(func=printinfo)

    parser = subparsers.add_parser('dot', help='Generate the DOT file representing the connectivity graph of the robot model')
    setRobotArgs(parser)
    parser.add_argument('-o', '--out-file', dest='outdot', metavar='FILE', default='robot.dot', help='the output .dot file')
    parser.set_defaults(func=writeDOTFile)

    parser = subparsers.add_parser('motdsl', help='Generate the Motion-DSL model corresponding to the kinematics of the robot model')
    setRobotArgs(parser)
    parser.add_argument('-o', '--out-file', dest='outmotdsl', metavar='FILE', help='the output .motdsl file - defaults to stdout')
    parser.set_defaults(func=writeMotDSLFile)

    parser = subparsers.add_parser('exp', help='Export the input model to a different format (experimental - work in progress)')
    setRobotArgs(parser)
    parser.add_argument('-f', '--format',   dest='oformat', metavar='FMT', help='desired output format: {yaml,kindsl,urdf} (default: yaml)')
    parser.add_argument('-o', '--out-file', dest='outfile', metavar='FILE', help='the output file - defaults to stdout')
    parser.set_defaults(func=export)

    parser = subparsers.add_parser('debug')
    setRobotArgs(parser)
    parser.set_defaults(func=playground)

    args = argparser.parse_args()
    if args.verbose :
        log.setLevel(logging.DEBUG)

    if hasattr(args, 'func') :
        args.func(args)
    else :
        argparser.print_usage()
