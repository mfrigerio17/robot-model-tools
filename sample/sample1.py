'''
Demonstrate the usage of part of the API of the `robmodel` package.

The `main()` function loads the sample UR5 models shipped with this project,
in different formats, and for each model prints the calculated pose of the last
link of the manipulator relative to the base. If the models are kinematically
equivalent, the results should be all the same.
'''

import os, logging
import numpy as np

import kgprim.core as primitives
import kgprim.motions

import robmodel
import robmodel.treeutils
import robmodel.convert.urdf.imp as urdfin
import robmodel.convert.kindsl.imp as kindslin
import robmodel.convert.yaml.imp as yamlin

import kgprim.ct.frommotions as motionconvert
import kgprim.ct.repr.mxrepr as mxrepr


logger = logging.getLogger(__package__)


def base_H_ee(geometryModel, eelinkname):
    '''Compute the homogeneous transformation matrix from the frame of the link
    given in the argument to the robot base, for the default configuration of
    the robot.'''

    robot  = geometryModel.connectivityModel
    tree   = robmodel.treeutils.TreeUtils( robot )
    frames = geometryModel.framesModel

    if eelinkname not in robot.links :
        logger.error("Could not find link '{0}' on robot '{1}'".format(eelinkname, robot.name) )
        return None

    ee = robot.links[ eelinkname ]
    H  = np.identity(4)

    currentLink = ee
    while currentLink is not None :
        parent = tree.parent( currentLink )
        if parent is not None :
            joint = tree.connectingJoint(currentLink, parent)
            frJ  = frames.jointFrames[ joint ]
            frP  = frames.linkFrames[ parent ]
            pose = primitives.Pose(target=frJ, reference=frP)
            posespec = geometryModel.byPose[ pose ]

            parent_CT_link = motionconvert.toCoordinateTransform(posespec)
            parent_H_link  = mxrepr.hCoordinatesNumeric(parent_CT_link)
            H = parent_H_link @ H
            #print( pose, "\n")
            #print( np.round(link_H_parent,5), "\n", np.round(H[:3,3], 5), "\n" )
        currentLink  = parent

    return H




if __name__ == '__main__':
    formatter = logging.Formatter('%(levelname)s : %(message)s')
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.setLevel(logging.WARN)
    logger.addHandler(handler)

    eeLink = "wrist_3"

    here = os.path.dirname(__file__)

    urdffile = open( os.path.join(here, "models/ur5/ur5.urdf") )
    urdf = urdfin.URDFWrapper(urdffile)
    c, o, f, g = urdfin.convert(urdf)
    H = base_H_ee(g, eeLink)
    if H is None :
        logger.error("Cannot determine EE transform from the URDF model")
    else :
        print("\nWrist 3 default pose from the URDF model:")
        print( np.round(H, 5) )

    kindslfile = os.path.join(here, "models/ur5/ur5.kindsl")
    c, o, f, g, i = kindslin.convert(kindslfile)
    H = base_H_ee(g, eeLink)
    if H is None :
        logger.error("Cannot determine EE transform from the KinDSL model")
    else :
        print("\nWrist 3 default pose from the KinDSL model:")
        print( np.round(H, 5) )

    istream = open( os.path.join(here, "models/ur5/ur5-connect.yaml") )
    connectivity = yamlin.connectivity( istream )
    istream.close()

    istream = open( os.path.join(here, "models/ur5/ur5-numbering.yaml") )
    nscheme = yamlin.numbering_scheme(istream)
    istream.close()

    robot  = robmodel.ordering.Robot( connectivity, nscheme )
    frames = robmodel.frames.RobotDefaultFrames(robot, [])

    istream = open(os.path.join(here, "models/ur5/ur5-geometry.yaml") )
    metric  = yamlin.geometry(istream)
    istream.close()

    full = robmodel.geometry.Geometry(robot, frames, metric)
    H = base_H_ee(full, eeLink)
    if H is None :
        logger.error("Cannot determine EE transform from the YAML model")
    else :
        print("\nWrist 3 default pose from the YAML model:")
        print( np.round(H, 5) )

