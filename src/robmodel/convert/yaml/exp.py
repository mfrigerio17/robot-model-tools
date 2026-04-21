import sys
from mako.template import Template

import kgprim
from kgprim.motions import MotionStep
from kgprim.motions import Axis

import robmodel.geometry
from robmodel.frames import jointFrameName
from robmodel.frames import linkFrameName
from robmodel.convert.utils import FloatsFormatter

model_kind_to_file_name_defaults = {
"connectivity": "connectivity.yaml",
"numbering":    "numbering.yaml",
"geometry":     "geometry.yaml",
"inertia":      "inertia.yaml",
"joint_limits": "joint-limits.yaml",
"user_frames" : "user_frames.yaml"
}

tpl_index = Template('''
model: ${model.name}

%for modelname, file in model_files.items():
${modelname} : "${file}"
%endfor
''')

tpl_connectivity = Template('''
model: ${connectivity.name}

links:
%for name in connectivity.links:
  - ${name}
%endfor

joints:
%for name, joint in connectivity.joints.items():
  - name: ${name}
    kind: ${joint.kind.name}
%endfor

pairs:
%for joint in connectivity.joints.values():
  - joint: ${joint.name}
    link1: ${connectivity.jointToLinkPair(joint)[0].name}
    link2: ${connectivity.jointToLinkPair(joint)[1].name}
%endfor
''')


tpl_numbering = Template('''
model: ${model.name}

nums:
%for name, link in model.links.items():
  ${name} : ${model.linkNum(link)}
%endfor

''')

tpl_geometry = Template('''
model: ${geometry.robotName}

mode: currentFrame

poses:
%for joint in geometry.connectivityModel.joints.values():
  - target: ${jointFrameName(joint)}
    reference: ${linkFrameName(geometry.connectivityModel.predecessor(joint))}
    steps:
%    for seq in geometry.byJoint[joint].motion.sequences:
%        for step in seq.steps:
      - kind: ${stepKind(step)}
        amount: ${fmt.float2str(step.amount, angle=stepIsRot(step))}
%        endfor
%    endfor
%endfor
%for name, frame in geometry.framesModel.userFrames.items():
<%  posespec = relativePose(frame) %>
%   if posespec :
  - target: ${name}
    reference: ${linkFrameName(frame.body)}
    steps:
%    for seq in posespec.motion.sequences:
%        for step in seq.steps:
      - kind: ${stepKind(step)}
        amount: ${fmt.float2str(step.amount, angle=stepIsRot(step))}
%        endfor
%    endfor
%   endif
%endfor
''')

tpl_inertia = Template('''
model: ${inertia.robot.name}

inertia:
%for link, ip in inertia.inertia.items():
  ${link} :
    mass: ${fmt.float2str(ip.mass)}
    com:
      frame: ${ip.com.frame.name}
      x : ${fmt.float2str(ip.com.x)}
      y : ${fmt.float2str(ip.com.y)}
      z : ${fmt.float2str(ip.com.z)}
    moments:
      frame : ${ip.moments.frame.name}
      ixx : ${fmt.float2str(ip.moments.ixx)}
      iyy : ${fmt.float2str(ip.moments.iyy)}
      izz : ${fmt.float2str(ip.moments.izz)}
      ixy : ${fmt.float2str(ip.moments.ixy)}
      ixz : ${fmt.float2str(ip.moments.ixz)}
      iyz : ${fmt.float2str(ip.moments.iyz)}

%endfor
''')


tpl_jointlimits = Template('''
model : ${limits.connectivityModel.name}

limits:
%for name, joint in limits.connectivityModel.joints.items():
<% jdata = limits.byJoint[joint] %>
    ${name}:
%if jdata.lower_pos :
        lower_pos: ${jdata.lower_pos}
%endif
%if jdata.upper_pos :
        upper_pos: ${jdata.upper_pos}
%endif
%if jdata.velocity :
        velocity: ${jdata.velocity}
%endif
%if jdata.force :
        force: ${jdata.force}
%endif
%endfor
''')


tpl_userframes = Template('''
model: ${frames.robot.name}

userframes:
%for name, frame in frames.userFrames.items():
    - name: ${name}
      attached_to: ${frame.body.name}

%endfor
''')

_formatter = FloatsFormatter()

def indexFileText(robot, model_to_file_map):
    text = tpl_index.render(model=robot, model_files=model_to_file_map)
    return text

def connectivityModelText(connectivity_model):
    text = tpl_connectivity.render(connectivity=connectivity_model)
    return text

def orderingModelText(orderingModel):
    text = tpl_numbering.render(model=orderingModel)
    return text


__ser_map = {
    MotionStep.Kind.Translation : {
        Axis.X : 'trx',
        Axis.Y : 'try',
        Axis.Z : 'trz',
    },
    MotionStep.Kind.Rotation : {
        Axis.X : 'rotx',
        Axis.Y : 'roty',
        Axis.Z : 'rotz',
    },
}

def geometryModelText(geometryModel):
    text = tpl_geometry.render(geometry=geometryModel,
        jointFrameName= lambda j    : jointFrameName(geometryModel.connectivityModel, j),
        linkFrameName = lambda l    : linkFrameName(geometryModel.connectivityModel, l),
        stepIsRot     = lambda step : step.kind==MotionStep.Kind.Rotation,
        stepKind      = lambda step : __ser_map[step.kind][step.axis],
        relativePose  = lambda frame : robmodel.geometry.getPoseSpec(geometryModel, frame),
        fmt = _formatter
    )
    return text

def inertiaModelText(inertia_model):
    text = tpl_inertia.render(inertia=inertia_model, fmt=_formatter)
    return text

def jointLimitsModelText(joint_limits_model):
    text = tpl_jointlimits.render(limits=joint_limits_model)
    return text

def userFramesModelText(geometry_model):
    text = tpl_userframes.render(frames=geometry_model.framesModel)
    return text

