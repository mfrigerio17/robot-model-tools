#include <iostream>
#include <meshcatcpp/buffer.h>

void meshcat::PackedDataBuffer::write(const char* src, size_t n)
{
    if(cursor+n > buf_size) {
        return;// TODO should not be silent
    }
    memcpy( &(buffer[cursor]), src, n);
    cursor = cursor+n;
}
