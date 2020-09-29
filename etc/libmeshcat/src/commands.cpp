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


meshcat::PckSetTransform::PckSetTransform() : buf_path(buf_full), buf_matrix(buf_full)
{
    // There is no need to repack a fixed thing every time; so we do it once
    // here
    msgpack::pack(buf_full, topack_cmd);
    packed_header_size = buf_full.size();  // used later to move the cursor in the buffer

    // Overwrite the header byte with 0x83, meaning a dictionary with three
    // entries. We are cheating and we will add the other two entries later
    buf_full.set_first_byte(131); //
}


const meshcat::PackedDataBuffer&
meshcat::PckSetTransform::pack_payload(
        const std::string& meschat_path, const double* matrix_data)
{

    buf_full.reset(packed_header_size);
    buf_path.reset();
    buf_matrix.reset();
    // copy the given input data and msgpack it
    // keep in mind that packing into `buf_path` and `buf_matrix` actually
    //  writes data in `buf_full`
    topack_path["path"] = meschat_path;
    memcpy(mxdata.data(), matrix_data, sizeof(double)*16);
    msgpack::pack(buf_path,   topack_path);
    msgpack::pack(buf_matrix, topack_matrix);

    return buf_full;
}

