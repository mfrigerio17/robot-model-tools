#ifndef MESHCAT_CPP_BRIDGE_DATABUFFER_H
#define MESHCAT_CPP_BRIDGE_DATABUFFER_H



namespace meshcat {


struct PackedDataBuffer
{
    static constexpr size_t buf_size = 1024;

    using raw_buff_t = unsigned char[buf_size];

    void write(const char* src, size_t n) {
        if(cursor+n > buf_size) {
            return;// TODO should not be silent
        }
        memcpy( &(buffer.whole[cursor]), src, n);
        cursor = cursor+n;
    }

    const raw_buff_t& data() const { return buffer.whole; }
    const unsigned char* data_skip_first() const { return buffer.partitioned.except_first; }

    size_t size() const { return cursor; }

    void set_first_byte(unsigned char val) { buffer.partitioned.first = val; }
    void reset(size_t where=0) { cursor = where; }


    size_t cursor = 0;

    struct _buf {
        unsigned char first;
        unsigned char except_first[buf_size-1];
    };
    union buf {
        raw_buff_t whole;
        _buf partitioned;
    };

    buf buffer;

};



}


#endif
