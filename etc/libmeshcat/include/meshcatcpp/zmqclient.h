#ifndef MESHCAT_CPP_BRIDGE_ZMQCLIENT_H
#define MESHCAT_CPP_BRIDGE_ZMQCLIENT_H

#include <string>
#include <zmq.hpp>
#include "buffer.h"

namespace meshcat {

const std::string default_zmq_url("tcp://127.0.0.1:6000");


struct ZMQClient
{
    ZMQClient(zmq::context_t& context) : mysock(context, zmq::socket_type::req) {}

    void connect(const std::string& url);
    void send_command(const std::string& cmd, const std::string& path, const PackedDataBuffer& message);

    zmq::socket_t mysock;
};


}


#endif
