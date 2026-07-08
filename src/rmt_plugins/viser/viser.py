import pathlib
import numpy as np
import kgprim.ct.frommotions as frommotions
import kgprim.ct.repr.mxrepr as mxrepr
import kgprim.ct.metadata

import robmodel.jposes
import robmodel.frames
import robmodel.treeutils as treeu
from robmodel.frames import FrameRole
import viser, viser.transforms, trimesh


def makeServer():
    server = viser.ViserServer()

    server.gui.configure_theme(
        control_width="large",
        dark_mode=True,
        show_logo=False,
        show_share_button=False
    )

    server.scene.add_grid(
        "/grid",
        width=2,
        height=2,
        position=(0.0, 0.0, 0.0),
    )
    return server

class ViserScene:
    def __init__(self, robotGeometryModel, viserServer, mesheFilesPaths={}):
        self.robotGeometry = robotGeometryModel
        self.robot = robotGeometryModel.connectivityModel
        self.treeutils = treeu.TreeUtils(self.robot)
        self.jointPoses = robmodel.jposes.JointPoses(
            robotGeometryModel.connectivityModel,
            robotGeometryModel.framesModel,
            robotGeometryModel.jointAxes)
        self.jointTransforms = {}
        self.mesheFilesPaths = mesheFilesPaths

        self.scene = viserServer.scene
        self.server = viserServer
        self.h_links   = {}
        self.h_jframes = {}
        self.h_lframes = {}
        self.h_uframes = {} # handles for the user-frames (extra frames attached to links)
        self.h_meshes  = {}
        self.h_jaxes   = {}
        self.visual_attrs = {
            "axes_radius" : 0.005,
            "axes_length" : 0.25,
            "jointF_to_linkF_ratio": 0.8,
            "joint_color" : (255,0,255),
            "link_color" : (255,255,0),
            "jaxis_color" : (100,100,100),
            "jaxis_length": 0.3,
        }


    def _getNumericalMatrix(self, pose_spec):
        matrix = None
        ctransform = frommotions.toCoordinateTransform(pose_spec)
        ct_info = kgprim.ct.metadata.TransformMetadata(ctransform)
        if ct_info.is_parametric :
            #if self.robotGeometryParams is None :
            raise RuntimeError("Cannot resolve parametric transforms without parameter values")

            matrix_with_symbols = mxrepr.hCoordinatesSymbolic(ctransform)
            pvalues = {}
            for p in ct_info.parameters: # this is a ordered set
                pvalues[p] = self.robotGeometryParams[p.name]
            matrix_with_symbols.setParametersValue(pvalues)
            matrix = matrix_with_symbols.eval().astype(float)
        else:
            matrix = mxrepr.hCoordinatesNumeric(ctransform).astype(float)
        return matrix

    def _addMesh(self, name, fullScenePath):
        meshpath = self.mesheFilesPaths.get(name, None)
        if meshpath:
            mesh = trimesh.load_mesh(meshpath)
            if mesh:
                self.h_meshes[name] = self.scene.add_mesh_simple(
                    fullScenePath, vertices=mesh.vertices, faces=mesh.faces)
#                self.h_meshes[link.name] = self.scene.add_mesh_trimesh(
#                    fullScenePath, mesh=mesh)

    def _addToScene(self, link, parent, scenePathToParent):
        '''
        The local visualization tree that we have to construct:
         parent \
                |- joint \
                         |- joint frame
                         |- link \
                                 |- link frame
                                 |- mesh
        '''
        vattrs = self.visual_attrs
        scenePath = scenePathToParent
        if parent is None : # the base link of the robot; the scene parent is thus the root element
            H = np.identity(4)
        else :
            joint = self.robot.linkPairToJoint(link, parent)

            # Geometrical data, ie the fixed pose of the joint frame
            # relative to the predecessor link frame
            H = self._getNumericalMatrix( self.robotGeometry.byJoint[joint] )

            scenePath = scenePath+"/"+joint.name
            # joint placeholder
            se3 = viser.transforms.SE3.from_matrix(matrix=H)
            self.scene.add_icosphere(scenePath, radius=0.005,
                position=se3.translation(), wxyz=se3.rotation().wxyz,
                color=vattrs["joint_color"], visible=True)
            # joint frame
            self.h_jframes[joint.name] = self.scene.add_frame(
                scenePath+"/frame",
                scale= vattrs["jointF_to_linkF_ratio"],
                axes_length= vattrs["axes_length"],
                axes_radius= vattrs["axes_radius"],
                origin_color= vattrs["joint_color"],
                visible=False)

            # joint axis arrow
            if joint.name in self.robotGeometry.jointAxes:
                tip = np.array(self.robotGeometry.jointAxes[joint.name]) * vattrs["jaxis_length"]
                self.h_jaxes[joint.name] = self.scene.add_arrows(
                    scenePath+"/axis",
                    points= np.array([[[0,0,0],tip]]),
                    colors= vattrs["jaxis_color"],
                    shaft_radius= vattrs["axes_radius"],
                    head_length= vattrs["jaxis_length"] *0.1,
                    head_radius= vattrs["axes_radius"]*1.5,
                    visible=False)

            tr = self.jointPoses.jointTransform(joint)
            tr = mxrepr.hCoordinatesSymbolic(tr)
            self.jointTransforms[joint.name] = tr;
            if len(tr.variables) > 0 :
                H = tr.eval(0.0)
            else:
                H = tr.eval()

        #link placeholder
        scenePath = scenePath+"/"+link.name
        se3 = viser.transforms.SE3.from_matrix(matrix=H)
        self.h_links[link.name] = self.scene.add_box(scenePath,
            dimensions=(0.01,0.01,0.01),
            color=vattrs["link_color"], position=se3.translation(),
            wxyz=se3.rotation().wxyz, visible=True)
        # link frame
        self.h_lframes[link.name] = self.scene.add_frame(scenePath+"/frame",
                scale= 1,
                axes_length= vattrs["axes_length"],
                axes_radius= vattrs["axes_radius"],
                origin_color= vattrs["link_color"],
                visible=False)
        # other frames attached to the same link
        fmodel = self.robotGeometry.framesModel
        for fr in fmodel.attachedTo(link):
            if fmodel.frameRole(fr) == FrameRole.user:
                H = self._getNumericalMatrix( self.robotGeometry.getPoseSpec(fr) )
                se3 = viser.transforms.SE3.from_matrix(matrix=H)
                # a placeholder, to have a common handle at the right pose
                self.scene.add_icosphere(f"{scenePath}/uframes/{fr.name}", radius=0.0,
                    position=se3.translation(), wxyz=se3.rotation().wxyz,
                    color=vattrs["joint_color"], visible=True)
                self.h_uframes[fr.name] = self.scene.add_frame(
                    f"{scenePath}/uframes/{fr.name}/axes",
                    scale= vattrs["jointF_to_linkF_ratio"],
                    axes_length= vattrs["axes_length"],
                    axes_radius= vattrs["axes_radius"],
                    origin_color= vattrs["link_color"],
                    visible=False)
                # if there is a mesh with a corresponding name, load it
                self._addMesh(fr.name, f"{scenePath}/uframes/{fr.name}/mesh")

        # the link mesh
        self._addMesh(link.name, scenePath+"/mesh")

        for child in self.treeutils.children(link) :
            self._addToScene(child, link, scenePath)

    def loadRobotIntoScene(self):
        '''
        Loads the robot model (links, frames, meshes) and the related
        GUI controls
        '''

        # First add the elements of the 3D scene
        self._addToScene(self.robot.base, None, "/")

        # Then the GUI controls.
        # `event` (below) is a Viser GUI object. The `target` is the affected gui element

        gui = self.server.gui
        with gui.add_folder(label="Frames & Co."):
            opts = ["show","hide"]
            group = self.server.gui.add_tab_group()
            def addTabWithFramesCtrls(label, rob_items, frames_h):
                with group.add_tab(label):
                    h_buttons = gui.add_button_group(label="All:", options=opts)
                    checkboxes = []
                    def _showFrame(event) : # handler for a single checkbox
                        frames_h[event.target.label].visible = event.target.value
                    for name in rob_items:
                        h = gui.add_checkbox(label=name, initial_value=frames_h[name].visible)
                        h.on_update(_showFrame)
                        checkboxes.append(h)
                    def _setAll(event): # handler for the buttons
                        for box in checkboxes : box.value = (event.target.value==opts[0])
                        # note that in Viser setting a checkbox triggers in turn its side effect
                    h_buttons.on_click(_setAll)
            addTabWithFramesCtrls("Link frames" , self.robot.links,  self.h_lframes)
            addTabWithFramesCtrls("Joint frames", self.robot.joints, self.h_jframes)
            addTabWithFramesCtrls("Joint axes", self.robot.joints, self.h_jaxes)
            addTabWithFramesCtrls("Others", self.robotGeometry.framesModel.userFrames, self. h_uframes)


        with self.server.gui.add_folder(label="Meshes"):
            h = self.server.gui.add_slider(label="opacity", min=0.0, max=1.0, initial_value=1.0, step=0.1)
            h.on_update(func= lambda ev:
                self._setOpacity(self.h_meshes.values(), ev.target.value))

        with self.server.gui.add_folder(label="Explode"):
            h = self.server.gui.add_slider(label="distance", min=0, max=1, initial_value=0, step=0.05)
            h.on_update(func= lambda ev:
                self._setExplodeDistance(self.h_links.values(), ev.target.value))

    def _setVisibility(self, handles, visible):
        for h in handles : h.visible = visible

    def _setOpacity(self, handles, opacity):
        for h in handles : h.opacity = opacity

    def _setExplodeDistance(self, handles, dist):
        for h in handles :
            linkName = pathlib.Path(h.name).name # extracts the last part
            link  = self.robot.links[linkName]
            joint = self.treeutils.supportingJoint(link)
            if joint is not None:
                tr = self.jointTransforms[joint.name]
                H = tr.eval(0.0) if len(tr.variables) > 0 else tr.eval() #TODO
                pos = H[0:3,3]
                pos[2] = pos[2] + dist
                h.position = pos
