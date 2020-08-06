import sys
from mako.template import Template

tpl = Template('''
robot: ${robot_name}

inertia:
%for link, ip in inertia.items():
  ${link} :
    mass: ${ip.mass}
    com:
      frame: ${ip.com.frame.name}
      x : ${ip.com.x}
      y : ${ip.com.y}
      z : ${ip.com.z}
    moments:
      frame : ${ip.moments.frame.name}
      ixx : ${ip.moments.ixx}
      iyy : ${ip.moments.iyy}
      izz : ${ip.moments.izz}
      ixy : ${ip.moments.ixy}
      ixz : ${ip.moments.ixz}
      iyz : ${ip.moments.iyz}

%endfor
''')

def inertia(inertia_model, out_stream=sys.stdout):
    text = tpl.render( robot_name=inertia_model.robot.name, inertia=inertia_model.inertia )
    out_stream.write( text )
