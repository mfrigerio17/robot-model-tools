This is the readme file of the "Robot Model Tools" Python package.

The modules in the `robmodel` package allow to model some aspects of articulated
robots, such as connectivity, numbering scheme of the links, attached frames,
geometry, etc.

The `rmt` package implements a command line program that operates on robot
models, such as the URDF or the KinDSL formats. See

```
rmt --help
```

for a list of the available sub commands.

You can try the program without installing this repository
(although you still need to install the dependencies):
```
cd src/
./runner.py --help
```

# Code documentation

More information about the packages and the modules of this project is available
in the source code.

You can generate e.g. html documentation using [pdoc](https://pdoc.dev/).
For example:

```sh
cd src/
pdoc --no-show-source -o /tmp/docs/rmt rmt/ robmodel/
```

# Installation
```
pip install rmt
```

Alternatively, install from the source code:

```sh
git clone <repo> rmt    # replace <repo> with the right URL
cd rmt/
pip install .           # should also install the dependencies available in PyPI
```


#### Virtual environment
You might want to install the tool and its dependencies in an isolated virtual
environment. If so, run the following before the installation procedure:

```sh
mkdir myvenv && python3 -m venv myvenv
source myvenv/bin/activate  # may need to pick another script depending on your shell
#pip install wheel           # may also required to set up the env
```

## Dependencies
The following libraries are used by this project:

- [kgprim](https://github.com/mfrigerio17/kgprim) kinematics/geometric primitives

- [NetworkX](http://networkx.github.io/), for the connectivity model, which is a
  graph

- [NumPy](http://www.numpy.org)

Optionally:

- [textX](http://textx.github.io/textX/stable/), for the KinDSL format importer

- [PyYAML](http://pyyaml.org), to import YAML models

- [Mako](https://www.makotemplates.org/), for the export of models to other
  formats (which involves text generation)

- [PyGraphviz](https://pygraphviz.github.io/), required only by the DOT file
  generator command of the `rmt` tool


# License

Copyright © 2019-2022, Marco Frigerio

Released under the BSD 3-clause license. See the `LICENSE` file for additional
information.

