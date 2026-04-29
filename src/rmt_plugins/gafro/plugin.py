import rmt_plugins.gafro as this
import rmt_plugins.gafro.imp

from rmt_plugins.gafro import logger

model_format_id = "gafro"
model_file_extension = "yaml"


def load_models(filepath, **kwargs):
    istream = open(filepath)
    connectivity, numbering, frames, geometry, inertia, limits = this.imp.import_model(istream, **kwargs)
    istream.close()
    return connectivity, numbering, frames, geometry, inertia, None, limits
