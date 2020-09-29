#ifndef MESHCAT_CPP_BRIDGE_COMMANDS_H
#define MESHCAT_CPP_BRIDGE_COMMANDS_H

#include <string>
#include "buffer.h"


namespace meshcat {

extern std::string cmd_set_transform;


struct PckSetTransform
{
    PckSetTransform();

    const PackedDataBuffer&
    pack_payload(const std::string& path, const double* matrix_data);

    PackedDataBuffer buf_full;
    TrickyBuffer     buf_path;
    TrickyBuffer     buf_matrix;

private:
    size_t packed_header_size;
};



}


#endif
