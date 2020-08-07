import os, logging

import robmodel.convert.urdf.exp as urdfout
import robmodel.convert.json as jsonconvert

import robmodel.connectivity

import json
import jsonschema
from jsonschema.exceptions import ValidationError
from pyld import jsonld



logger = logging.getLogger(__name__)

def _load_schema(filename):
    ifile   = os.path.join(jsonconvert.__path__[0], 'schemas', filename)
    istream = open(ifile)
    schema = json.load(istream)
    istream.close()
    return schema

schema_connectivity = _load_schema('connectivity.json')
ctx_connectivity = _load_schema('connectivity.ctx')


def pp(raw) :
    print(json.dumps(raw, indent=2))

def ldload(istream):
    loaded = json.load(istream)
    ctx    = loaded['@context']

    # compact against empty context, to expand the terms; yes, compact to expand,
    # probably there is a better way to do this...
    compacted = jsonld.compact( loaded, {} )
    return loaded, ctx, compacted


def ldloadfile(filename):
    istream = open(filename)
    doc, ctx, compact = ldload(istream)
    istream.close()
    #pp(compact)
    return doc, ctx, compact


def validate_schema(doc, schema):
    # at the moment, the schema is designed to match documents with expanded
    # terms, i.e., models which are normalized and use "our" terminology
    try:
        jsonschema.validate(instance=doc, schema=schema)
    except ValidationError as e :
        logger.error("Failed to validate model: {0}".format(e.message))
        return False
    return True


def connectivity(istream):
    '''
    Import a connectivity model (`robmodel.connectivity.Robot`) from JSON-LD

    Arguments:
    - `istream` and input stream from which to read the JSON-LD text
    '''
    doc, ctx, compact = ldload(istream)
    valid = validate_schema(compact, schema_connectivity)
    if not valid :
        logger.error("JSON schema validation failure")
        return None

    # after all the validations, compact it again against our custom schema,
    # which normalizes the terms to minimize the further processing we have to
    # do
    compact = jsonld.compact( doc, ctx_connectivity )
    #pp(compact)

    # Now, finally, manually perform the remaining conversions required to
    # match the expected format of the dictionary that can be imported.
    # This all depends on such format, which is implicit in the code of the
    # robmodel.connectivity module
    links  = [ el["@id"] for el in compact['links'] ]
    joints = [ {'name': el['@id'], 'kind':el['kind'] } for el in compact['joints'] ]
    #print(links)
    #print(joints)
    data = {
        'name': compact['name']['@id'],
        'links' : links,
        'joints': joints,
        'pairs' : compact['pairs'] # this one is already conforming, no need to massage it
    }
    return robmodel.connectivity.fromDict(data)



def main():
    modelf  = 'sample/models/ur5/connectivity.jsonld'
    istream = open(modelf)
    connectivity_model = connectivity(istream)
    print(connectivity_model)

    # I cannot yet convert to URDF as I still need a numbering scheme for the
    # connectivity model, to establish the parentship relations which are
    # required for a bare minimum URDF
    #converted = urdfout.ordering(connectivity_model)
    #print(converted)

if __name__ == '__main__':
    main()

# TODO missing checks, perhaps possible with the context/schema:
# - the entries in the kinematic pairs must be links/joints defined previously




