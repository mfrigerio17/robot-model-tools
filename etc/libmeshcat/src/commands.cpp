#include <iostream>
#include <map>
#include <msgpack.hpp>
#include <meshcatcpp/commands.h>


std::string meshcat::cmd_set_transform = u8"set_transform";


static std::map<std::string, std::string> topack_cmd {
   { "type",  meshcat::cmd_set_transform }
};

static std::map<std::string, std::string> topack_path {
        { "path", u8"replace_me" }
};

static std::array<double, 16> mxdata;
static std::map<std::string, const std::array<double, 16>& > topack_matrix {
    { "matrix",  mxdata }
};


meshcat::PckSetTransform::PckSetTransform()
{
    // There is no need to repack a fixed thing every time; so we do it once
    // here
    msgpack::pack(full, topack_cmd);
    packed_header_size = full.size();  // used later to move the cursor in the buffer

    // Overwrite the header byte with 0x83, meaning a dictionary with three
    // entries. We are cheating and we will add the other two entries later
    full.set_first_byte(131); //
}


static void concat_buffers(
    meshcat::PackedDataBuffer& cmd,
    const meshcat::PackedDataBuffer& path,
    const meshcat::PackedDataBuffer& matrix)
{
    cmd.write( (char*)  path.data_skip_first(),   path.size()-1 );
    cmd.write( (char*)matrix.data_skip_first(), matrix.size()-1 );
}


void meshcat::PckSetTransform::pack_payload(const std::string& meschat_path, const double* matrix_data)
{
    memcpy(mxdata.data(), matrix_data, sizeof(double)*16);

    full.reset(packed_header_size);
    path.reset();
    matrix.reset();
    topack_path["path"] = meschat_path;
    msgpack::pack(path,   topack_path);
    msgpack::pack(matrix, topack_matrix);

    concat_buffers(full, path, matrix);
}

