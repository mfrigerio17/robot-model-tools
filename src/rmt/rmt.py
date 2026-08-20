import os, sys, logging, argparse, traceback
import pathlib
import networkx as nx
import numpy as np

import robmodel.convert.urdf.imp as urdfin
import robmodel.connectivity
import robmodel.inertia
import robmodel.jposes
import robmodel.jlimits

import rmt.load
import rmt.kinematics
import kgprim.values

log = rmt.logger


def getmodels(filepath, paramsFilePath=None, jlimsFilePath=None, floatLiteralsAsConstants=False, **kwargs):
    connectivity = None
    ordering     = None
    frames       = None
    geometry     = None
    inertia      = None
    jlimits      = None
    _, ext = os.path.splitext(filepath)
    if ext == '.urdf' :
        log.debug("URDF format detected for file " + filepath)
        try:
            urdffile = open(filepath)
            urdfwrap = urdfin.URDFWrapper(urdffile)
            connectivity, ordering, frames, geometry, inertia, jlimits = urdfin.convert(urdfwrap, **kwargs)[0:6]
        except Exception as e:
            log.error("Failed to load URDF model: {} - {}".format(e.__class__.__name__, e))
            log.debug(traceback.format_exc(limit=-4))
            exit(-1)
        urdffile.close()

    elif ext == '.kindsl' :
        log.debug("KinDSL format detected for file " + filepath)
        try:
            import robmodel.convert.kindsl.imp as kindslin
        except ImportError as e:
            log.error("KinDSL support not available! " +
                      "Perhaps you miss 'textX' in the Python environment? "+
                      "The import error was: " + str(e))
            exit(-1)
        try:
            connectivity, ordering, frames, geometry, inertia = kindslin.convert(filepath, floatLiteralsAsConstants)
        except Exception as e:
            log.error("Failed to load KinDSL model: {}".format(str(e)))
            log.debug(traceback.format_exc(limit=-4))
            exit(-1)

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

            path = data.get('user_frames', None)
            istream = open(os.path.join(basepath, path)) if path else None
            frames = yamlin.frames(ordering, istream)
            if istream: istream.close()
            if 'geometry' in data:
                path = os.path.join(basepath, data['geometry'])
                istream = open(path)
                metric = yamlin.geometry(istream, floatLiteralsAsConstants)
                istream.close()
                geometry = robmodel.geometry.Geometry(ordering, frames, metric)
            if 'inertia' in data :
                filepath = os.path.join(basepath, data['inertia'])
                istream = open(filepath)
                inertia_data = yamlin.inertia(istream, robotFrames=frames, floatLiteralsAsConstants=floatLiteralsAsConstants);
                istream.close()
                if inertia_data == None :
                    raise RuntimeError("Could not load inertia data")
                inertia = robmodel.inertia.RobotLinksInertia(connectivity, frames, inertia_data)
        if 'joint_limits' in data:
            path = os.path.join(basepath, data['joint_limits'])
            with open(path) as istream:
                jlimits = yamlin.joint_limits(istream, connectivity)
    else :
        log.error("Unknown robot model extension '{0}'".format(ext))
        exit(-1)

    params = rmt.load.loadDictionary(paramsFilePath)

    if jlimsFilePath is not None:
        jlimits = rmt.load.jointLimits(jlimsFilePath, connectivity)

    return connectivity, ordering, frames, geometry, inertia, params, jlimits


def defpose(args, opts):
    robot,frames,geometry,_,paramsValues = getmodels(args.robot, **opts)[1:6]
    jointPoses = robmodel.jposes.JointPoses(robot, frames, geometry.jointAxes)
    kin = rmt.kinematics.RobotKinematics(geometry, jointPoses)
    H = rmt.kinematics.base_H_ee(kin, args.frame, paramsValues)
    if H is None :
        log.error("Could not compute the frame pose")
        exit(-1)
    print(np.round(H,5), file=args.ofstream)


def printinfo(args, opts):
    c,o,f,g,i = getmodels(args.robot, **opts)[0:5]
    print(c,o,f,g,i, file=args.ofstream)

def writeDOTFile(args, opts):
    connectivity, _, frames = getmodels(args.robot, **opts)[0:3]
    # Convert the graph to AGraph format used by pygraphviz
    ag = nx.nx_agraph.to_agraph( connectivity.graph )

    # Add the edge labels (joint names), to have them displayed
    for e in ag.edges() :
        e.attr['label'] = e.attr['joint']

    if args.colorfixed:
        for e in ag.edges():
            if connectivity.joints[e.attr['joint']].kind == robmodel.connectivity.JointKind.fixed:
                e.attr['color'] = 'deeppink'
                e.attr['penwidth'] = '3'

    if args.framesgraph:
        if frames is not None:
            for n in ag.nodes():
                link = connectivity.links[n.name]
                linkFrames = frames.attachedTo(link)
                label = '{' + '|'.join( [f.name for f in linkFrames] ) + '}'
                newname = n.name+"_FS"
                nframes = ag.add_node(newname, shape='record', label=label)
                ag.add_edge(n, newname)

    ag.write(args.ofstream)
    return ag


def writeMotDSLFile(args, opts):
    robot,frames,geometry = getmodels(args.robot, **opts)[1:4]
    jointPoses = robmodel.jposes.JointPoses(robot, frames, geometry.jointAxes)
    robotKin   = rmt.kinematics.RobotKinematics(geometry, jointPoses)
    rmt.kinematics.serializeToMotionDSLModel(robotKin, args.ofstream)




def _resolve_param(possiblyParametric, parametersValues, asFloat=False):
    '''
    Given a parametric expression, return the same expression after replacing
    the parameter with a constant, with value taken from the given dictionary.
    If there is no value in the dictionary, the parameter's default value is
    used; if that is also not available, log an error and return the original
    expression.
    '''
    val = possiblyParametric
    if isinstance(val, kgprim.values.Expression):
        if isinstance(val.argument, kgprim.values.Parameter):
            param = val.argument
            pname = param.name
            value = parametersValues.get(pname, param.defaultValue)
            if value is None:
                log.error("No value available for parameter '{}'".format(pname))
            else:
            # First create a Constant to replace the Parameter
            # Then replicate the same expression, but with the new argument
                cc   = kgprim.values.Constant("cc_{}".format(pname), value)
                expr = val.expr.subs({pname: cc.symbol})  # SymPy::subs()
                val  = kgprim.values.Expression(cc , expr)
                if asFloat :
                    val = float( val.evalf() )
    return val


def _resolve_parameters(poseSpecIterable, parametersValues):
    # Replace the parameters in geometry model with constants with the given
    # value. This allows the numerical evaluation of the geometry data, e.g.
    # when exporting the model to a format that does not support parameters.
    for poseSpec in poseSpecIterable:
        for motionSeq in poseSpec.motion.sequences:
            for step in motionSeq.steps:
                step.amount = _resolve_param(step.amount, parametersValues)

def _resolve_iparameters(inertiaModel, parametersValues):
    for linkName in inertiaModel.robot.links :
        inertia = inertiaModel.byLinkName(linkName)
        if inertia:
            inertia.mass  = _resolve_param(inertia.mass , parametersValues, asFloat=True)
            inertia.com.x = _resolve_param(inertia.com.x, parametersValues, asFloat=True)
            inertia.com.y = _resolve_param(inertia.com.y, parametersValues, asFloat=True)
            inertia.com.z = _resolve_param(inertia.com.z, parametersValues, asFloat=True)
            inertia.moments.ixx = _resolve_param(inertia.moments.ixx, parametersValues, asFloat=True)
            inertia.moments.ixy = _resolve_param(inertia.moments.ixy, parametersValues, asFloat=True)
            inertia.moments.ixz = _resolve_param(inertia.moments.ixz, parametersValues, asFloat=True)
            inertia.moments.iyy = _resolve_param(inertia.moments.iyy, parametersValues, asFloat=True)
            inertia.moments.iyz = _resolve_param(inertia.moments.iyz, parametersValues, asFloat=True)
            inertia.moments.izz = _resolve_param(inertia.moments.izz, parametersValues, asFloat=True)


def export(args, opts):
    c,o,f,g,i,params,jlimits = getmodels(args.robot, **opts)[0:7]
    oformat = args.oformat

    if oformat is None: oformat = 'yaml'

    try:
        if oformat == 'yaml' :
            import robmodel.convert.yaml.exp as yamlexp
            log.warning('Work-in-progress')
            if (args.output is None):
                raise RuntimeError("must pass some output folder when exporting to yaml")
            outpath = pathlib.Path(args.output).resolve()
            filenames = yamlexp.model_kind_to_file_name_defaults.copy()
            def mkfile(name, content):
                with open(outpath / name,  mode='w', encoding='utf-8', newline='\n') as ostream:
                    ostream.write(text)

            text = yamlexp.connectivityModelText(c)
            mkfile(filenames["connectivity"], text)

            text = yamlexp.orderingModelText(o)
            mkfile(filenames["numbering"], text)

            if g is not None:
                text = yamlexp.geometryModelText(g)
                mkfile(filenames["geometry"], text)

                text = yamlexp.userFramesModelText(g)
                mkfile(filenames["user_frames"], text)
            else:
                del filenames["geometry"]
                del filenames["user_frames"]

            if i is not None:
                text = yamlexp.inertiaModelText(i)
                mkfile(filenames["inertia"], text)
            else:
                del filenames["inertia"]

            if jlimits is not None:
                text = yamlexp.jointLimitsModelText(jlimits)
                mkfile(filenames["joint_limits"], text)
            else:
                del filenames["joint_limits"]

            text = yamlexp.indexFileText(c, filenames)
            mkfile(c.name+".yaml", text)

            return

        elif oformat == 'kindsl' :
            try:
                import robmodel.convert.kindsl.exp as kindslout
            except ImportError as e:
                log.error("KinDSL support not available!, are you perhaps missing textX in your Python environment?"+
                        " The import error was: " + e.msg())
                exit(-1)
            if g is not None:
                if params is not None:
                    # if parameter values were given, we assume the user wants
                    # to resolve parametrization
                    _resolve_parameters(g.posesModel.poses, params)
                    if i is not None :
                        _resolve_iparameters(i, params)
                text = kindslout.modelText(g,i)
            else :
                log.error("Sorry, I can export to KinDSL only a complete geometry model ")
                exit(-1)
        elif oformat == 'urdf' :
            import robmodel.convert.urdf.exp as urdfout
            if g is not None :
                _resolve_parameters(g.posesModel.poses, params)
                if i is not None:
                    _resolve_iparameters(i, params)
                extraPoses = None
                if args.extraposes is not None:
                    import motiondsl.motiondsl as motdsl
                    model = motdsl.dsl.modelFromFile(args.extraposes)
                    poseSpecModel = motdsl.toPosesSpecification(model)
                    _resolve_parameters(poseSpecModel.poses, params)
                    extraPoses = poseSpecModel.poses
                text = urdfout.modelText(g, i, jointLimits=jlimits,
                    userExtraPoses=extraPoses, includeDummies=not args.noextra)
            elif o is None:
                log.error("Cannot export a URDF if the input model does not even include ordering")
                exit(-1)
            else :
                text = urdfout.ordering(o)
        else :
            log.error("Unknown robot model format '{0}'".format(oformat))
            exit(-1)

        args.ofstream.write(text)
    except Exception as e:
        log.error("Could not export the robot model: {}".format(e))
        log.debug(traceback.format_exc())
        exit(-1)



def playground(args, opts):
    c,o,f,geometry,inertia,params,jlimits = getmodels(args.robot, **opts)[0:7]
    for link in c.links.values() :
        ip = inertia.byLink(link)
        if ip is not None:
            print(ip.mass)

    for pose in geometry.posesModel.poses :
        for m in pose.motion.steps:
            print(m)

    for name,limits in jlimits.byJoint.items():
        print(name,limits)


def setRobotArgs(argparser):
    argparser.add_argument('robot', metavar='robot-model', help='the robot model input file')
    argparser.add_argument('-p', '--params', dest='params', metavar='params-file', default=None, help='YAML/JSON file with default parameter values')
    argparser.add_argument('-j', '--joint-limits', dest='jlims', metavar='jlims-file', default=None, help='YAML/JSON file with joint limits data')
    argparser.add_argument('-b', '--base', dest='baseLink', metavar='NAME', help='consider the link named NAME as the root (defaults to the true root of the input model')
    argparser.add_argument('--ignore-fixed', dest='ignorefixed', action='store_true', help='ignore fixed joints when loading a model (might cause errors)')

def optsDict(parsed_arguments):
    return {
        'ignoreFixedJoints' : parsed_arguments.ignorefixed,
        'paramsFilePath': parsed_arguments.params,
        'jlimsFilePath' : parsed_arguments.jlims,
        'baseLinkName' : parsed_arguments.baseLink,
    }

def main():
    formatter = logging.Formatter('%(levelname)s (%(name)s) : %(message)s')
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger = logging.getLogger() # get the root one, to change settings across modules
    logger.setLevel(logging.WARN)
    logger.addHandler(handler)

    argparser = argparse.ArgumentParser(prog='rmt', description='Performs misceallaneous operations on robot models')
    argparser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help='lower the logging level to DEBUG')
    subparsers= argparser.add_subparsers()

    commonArgsParser = argparse.ArgumentParser(add_help=False)
    setRobotArgs(commonArgsParser)
    commonArgsParser.add_argument('-o', '--output', dest='output', metavar='FILE', help='output file/folder - defaults to stdout')

    parser = subparsers.add_parser('defpose', parents=[commonArgsParser], help="Print the pose of a frame relative to the base frame, assuming the zero-configuration of the robot")
    parser.add_argument('frame', metavar='frame', help='the name of the frame whose pose is of interest')
    parser.set_defaults(func=defpose)

    parser = subparsers.add_parser('print', parents=[commonArgsParser], help='Print some serialization of the robot model components')
    parser.set_defaults(func=printinfo)

    parser = subparsers.add_parser('dot', parents=[commonArgsParser], help='Generate the connectivity graph of the robot model in DOT format (requires pygraphviz)')
    parser.add_argument('-x', '--color-fixed', dest='colorfixed', action='store_true', help='use a color to highlight fixed joints')
    parser.add_argument('-f', '--include-frames', dest='framesgraph', action='store_true', help='include the list of frames attached to each link')
    parser.set_defaults(func=writeDOTFile)

    parser = subparsers.add_parser('motdsl', parents=[commonArgsParser], help='Generate the Motion-DSL model corresponding to the kinematics of the robot model')
    parser.set_defaults(func=writeMotDSLFile)

    parser = subparsers.add_parser('exp', parents=[commonArgsParser], help='Export the input model to a different format (experimental - work in progress)')
    parser.add_argument('-f', '--format',   dest='oformat', metavar='FMT', help='desired output format: {yaml,kindsl,urdf} (default: yaml)')
    parser.add_argument('-e', '--extra-poses',  dest='extraposes', metavar='FILE', help='add extra dummy joints/links to the exported URDF, for each pose in the given MotionDSL document')
    parser.add_argument('--no-extra',  dest='noextra', action='store_true', help='skip fixed joints for extra user frames when exporting (to URDF)')
    parser.set_defaults(func=export)

    parser = subparsers.add_parser('debug', parents=[commonArgsParser])
    parser.set_defaults(func=playground)

    args = argparser.parse_args()
    if args.verbose :
        logger.setLevel(logging.DEBUG)
    opts = optsDict(args)

    if hasattr(args, 'func') :
        openStream = args.output is not None
        if args.func == export:
            openStream = openStream and (args.oformat != 'yaml')
            # export to yaml interprets the output argument as a directory,

        if openStream:
            with open(args.output, mode='w', encoding='utf-8', newline='\n') as ostream:
                args.ofstream = ostream
                args.func(args, opts)
        else:
            # no output argument given, we default to stdout
            args.ofstream = sys.stdout # fine even when exporting yaml
            args.func(args, opts)
    else :
        argparser.print_usage()
