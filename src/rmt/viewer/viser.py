import pathlib
import numpy as np
import kgprim.ct.frommotions as frommotions
import kgprim.ct.repr.mxrepr as mxrepr
import kgprim.ct.metadata

import robmodel.jposes
import robmodel.frames
import robmodel.treeutils as treeu

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
        self.h_meshes  = {}
        self.visual_attrs = {
            "axes_radius" : 0.005,
            "axes_length" : 0.25,
            "jointF_to_linkF_ratio": 0.8,
            "joint_color" : (255,0,255),
            "link_color" : (255,255,0),
        }


    def _getNumericalMatrix(self, ctransform):
        matrix = None
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
            predecessor_P_joint  = self.robotGeometry.byJoint[joint]
            predecessor_CT_joint = frommotions.toCoordinateTransform(predecessor_P_joint)
            H = self._getNumericalMatrix(predecessor_CT_joint)

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
        # link mesh
        meshpath = self.mesheFilesPaths.get(link.name, None)
        if meshpath:
            mesh = trimesh.load_mesh(meshpath)
            if mesh:
                self.h_meshes[link.name] = self.scene.add_mesh_simple(
                    scenePath+"/mesh", vertices=mesh.vertices, faces=mesh.faces)
#                self.h_meshes[link.name] = self.scene.add_mesh_trimesh(
#                    scenePath+"/mesh", mesh=mesh)

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
        with gui.add_folder(label="Frames"):
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
            addTabWithFramesCtrls("Links" , self.robot.links,  self.h_lframes)
            addTabWithFramesCtrls("Joints", self.robot.joints, self.h_jframes)


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
