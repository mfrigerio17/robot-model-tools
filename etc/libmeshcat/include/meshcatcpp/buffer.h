#ifndef MESHCAT_CPP_BRIDGE_DATABUFFER_H
#define MESHCAT_CPP_BRIDGE_DATABUFFER_H

#include <string.h>

namespace meshcat {


struct PackedDataBuffer
{
    static constexpr size_t buf_size = 1024;
    using raw_buff_t = unsigned char[buf_size];

    void write(const char* src, size_t n);

    const raw_buff_t& data() const { return buffer; }

    size_t size() const { return cursor; }

    void set_first_byte(unsigned char val) { buffer[0] = val; }

    void reset(size_t where=0) { cursor = where; }


    size_t cursor = 0;
    raw_buff_t buffer;
};

struct TrickyBuffer
{
    TrickyBuffer(PackedDataBuffer& onto_me) :
        mywrite(&TrickyBuffer::write_skip_first), dest(onto_me) {}

    void write(const char* src, size_t n) {
        (this->*mywrite)(src, n);
    }

    void reset() {
        mywrite = &TrickyBuffer::write_skip_first;
    }

private:
    void write_normal(const char* src, size_t n) {
        dest.write(src, n);
    }
    void write_skip_first(const char* src, size_t n) {
        write_normal( src+1, n-1);
        mywrite = &TrickyBuffer::write_normal; // for the user's subsequent calls to write
    }

    void (TrickyBuffer::*mywrite)(const char* src, size_t n);
    PackedDataBuffer& dest;
};


}


#endif
