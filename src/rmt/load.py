import pathlib
from robmodel.jlimits import JointLimits


def loadDictionary(filePath):
    '''
    Load the dictionary encoded in the given YAML/JSON file.
    The empty dictionary is returned in case of any error.
    '''
    if filePath is None :
        return {}
    data = {}
    fpath = pathlib.Path(filePath)
    ext = fpath.suffix
    if ext == '.yaml' :
        import yaml
        with open(fpath) as istream:
            data = yaml.safe_load(istream)
    elif ext == '.json' :
        import json
        with open(fpath) as istream:
            data = json.load(istream)
    else:
        logger.error("Unknown extension '{}' for the dictionary file".format(ext))
    return data


def jointLimits(filePath, robotConnectivityModel):
    limits = loadDictionary(filePath)
    if len(limits) == 0:
        return None

    if limits["robot"] != robotConnectivityModel.name:
        logger.warning("Robot model name mismatch: '{}' (connectivity) vs '{}' (joint limits)"
            .format(robotConnectivityModel.name, limits["robot"]))

    return JointLimits(robotConnectivityModel, limits["limits"])



