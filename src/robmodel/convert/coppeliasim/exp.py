import logging
from mako.template import Template

import kgprim.ct as ct
import kgprim.ct.repr.mxrepr as mxrepr

import robmodel.convert.utils as utils

tpl = Template('''
local sim = require('sim')

local function create_model()

local hl, hj, hm, hroot = nil
local all_handles = { ilinks={}, ijoints={} }

local respondable_masks = { 65295, 65520 }
local rmask_i = 0

local function setup_link(link_name, supporting_joint_handle)
    hl = sim.importShape(0,
        "${robot.name}/meshes/"..link_name..".stl", --change the path if you already have a convex shape
        0, 0, 1)

    -- Apply Coppeliasim's default morphing into a convex decomposition
    -- Comment out if the imported mesh is fine already
    sim.convexDecompose(hl, 1, {1,500,200,0,0,0,0,0,0,0}, {100,30,0.25,0,0,0,0,0,0,0})

    sim.setObjectAlias(hl, link_name.."_dyn")
    if supporting_joint_handle ~= nil then -- will be nil for the base only
        sim.setObjectParent(hl, supporting_joint_handle);
    end
    sim.setObjectMatrix(hl, {1,0,0,0,0,1,0,0,0,0,1,0}, sim.handle_parent)
    sim.setObjectInt32Param(hl, sim.shapeintparam_static, 0)      -- i.e. make it dynamic...
    sim.setObjectInt32Param(hl, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(hl, sim.shapeintparam_respondable_mask, respondable_masks[rmask_i+1])

    -- Set a "default" inertia.
    -- Change this script generator to use the parameters from the source model, if
    -- available.
    sim.computeMassAndInertia(hl, 400)

    --Dynamic links do not normally have the special properties
    sim.setObjectSpecialProperty(hl,
        ~sim.objectspecialproperty_collidable &
        ~sim.objectspecialproperty_measurable &
        ~sim.objectspecialproperty_detectable)


    hm = sim.importShape(0,
        "${robot.name}/meshes/"..link_name..".stl",
        0, 0, 1)
    sim.setObjectAlias(hm, link_name.."_visible")
    sim.setObjectParent(hm, hl);
    sim.setObjectMatrix(hm, {1,0,0,0,0,1,0,0,0,0,1,0}, sim.handle_parent)
    sim.setObjectSpecialProperty(hm,
        sim.objectspecialproperty_collidable |
        sim.objectspecialproperty_measurable |
        sim.objectspecialproperty_detectable)

    -- Make the visible shape visibile in layer 1, the dynamic shape in layer 8
    sim.setObjectInt32Param(hm, sim.objintparam_visibility_layer, 1)
    sim.setObjectInt32Param(hl, sim.objintparam_visibility_layer, 256)

    all_handles[link_name] = hm
    all_handles[link_name.."_dyn"] = hl
    table.insert(all_handles.ilinks, hm)
end

<% link = robot.base %>
-- ---------- --
-- Robot base
-- ---------- --
setup_link("${link.name}", nil)
hroot = hl

-- set it as model, model base, with all properties enabled
sim.setModelProperty(hroot, 0)

sim.setObjectInt32Param(hroot, sim.shapeintparam_static, 1) -- the base is fixed

-- TODO: should set the option "set dynamic if it gets parent"
-- but I do not know how to do it with the API


% for joint in robot.joints.values() :
-- -------------- --
-- Joint ${joint.name}
-- -------------- --

hj = sim.createJoint(
    sim.joint_revolute_subtype,
    sim.jointmode_dynamic,
    0, nil)
sim.setObjectInt32Param(hj, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position)
sim.setObjectAlias(hj, "${joint.name}")

-- "connect" the joint with the predecessor link
sim.setObjectParent(hj, hl)
sim.setObjectMatrix(hj, ${jointPoseTable(joint)}, sim.handle_parent)

-- despite the name, this really sets the maximum force, when in position control mode
sim.setJointTargetForce(hj, 60)

-- joints are not normally visible
sim.setObjectInt32Param(hj, sim.objintparam_visibility_layer, 512)

all_handles.${joint.name} = hj
table.insert(all_handles.ijoints, hj)

<% link = robot.successor(joint) %>
-- -------------- --
-- Link ${link.name}
-- -------------- --
rmask_i = (rmask_i + 1) % 2
setup_link("${link.name}", hj)

% endfor

-- Store the handles "permanently" for convenient access
-- e.g. from the interactive Lua console in Coppeliasim
local buf = sim.packTable(all_handles)
sim.persistentDataWrite("${robot.name}_handles", buf)
-- Retrieve by
---- handles = sim.unpackTable(sim.persistentDataRead("${robot.name}_handles"))

return hroot

end

local function get_handles()
    local ret = {
        ilinks = {
% for link in robot.links.values() :
            [${robot.linkNum(link) + 1}] = sim.getObject("/${link.name}_visible"),
% endfor
        },
        links = {
% for link in robot.links.values() :
            ${link.name} = sim.getObject("/${link.name}_visible"),
% endfor
        },
        links_dyn = {
% for link in robot.links.values() :
            ${link.name} = sim.getObject("/${link.name}_dyn"),
% endfor
        },
        joints = {
% for joint in robot.joints.values() :
            ${joint.name} = sim.getObject("./${joint.name}"),
% endfor
        },
        ijoints = {
% for joint in robot.joints.values() :
            [${robot.jointNum(joint)}] = sim.getObject("./${joint.name}"),
% endfor
        },
    }

    return ret
end


return {
    create_model = create_model,
    get_handles = get_handles,
}
''')


def jointFramePoseCoefficients(geometryModel, joint):
    poseSpec = geometryModel.byJoint[ joint ]
    xt = ct.frommotions.toCoordinateTransform( poseSpec )
    H  = mxrepr.hCoordinatesNumeric(xt)

    values = []
    for c in [0,1,2]:
        values.extend( [H[c,0], H[c,1], H[c,2], H[c,3]] )
    return values

def listAsLuaTableCode(values, formatter) :
    text = ",".join([formatter.float2str(v) for v in values])
    return "{" + text + "}"

def code_creating_model(geometryModel):
    formatter = utils.FloatsFormatter()

    return tpl.render(
        robot = geometryModel.connectivityModel,
        jointPoseTable = lambda j : listAsLuaTableCode(
            jointFramePoseCoefficients(geometryModel, j), formatter)
    )

