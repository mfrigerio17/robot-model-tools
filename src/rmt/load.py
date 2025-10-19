import pathlib
import yaml
import json

from robmodel.jlimits import JointLimits


def jointLimits(filePath, robotConnectivityModel):
    limits = {}
    fpath  = pathlib.Path(filePath)
    ext    = fpath.suffix
    if ext == '.yaml' :
        import yaml
        with open(fpath) as istream:
            limits = yaml.safe_load(istream)

    elif ext == '.json' :
        import json
        with open(fpath) as istream:
            limits = json.load(istream)

    else:
        log.error("Unknown extension '{}' for the joint limits file".format(ext))
        return None

    if limits["robot"] != robotConnectivityModel.name:
        log.warning("Robot model name mismatch: '{}' (connectivity) vs '{}' (joint limits)"
            .format(robotConnectivityModel.name, limits["robot"]))

    return JointLimits(robotConnectivityModel, limits["limits"])
