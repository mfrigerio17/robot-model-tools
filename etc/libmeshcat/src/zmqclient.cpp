#include <iostream>
#include <meshcatcpp/zmqclient.h>

void meshcat::ZMQClient::connect(const std::string& url)
{
    try {
        mysock.connect(url);
    }
    catch(zmq::error_t& exc) {
        std::cerr << "Could not connect to ZMQ socket" << std::endl;
        std::cerr << exc.what() << std::endl;
        exit(-1);
    }
}


void meshcat::ZMQClient::send_command(
    const std::string& meshcat_command,
    const std::string& meshcat_path,
    const PackedDataBuffer& msgpacked )
{
    try {
        mysock.send( zmq::buffer(meshcat_command) , zmq::send_flags::sndmore);
        mysock.send( zmq::buffer(meshcat_path)    , zmq::send_flags::sndmore);
        mysock.send( zmq::buffer(msgpacked.data()) );
        zmq::message_t response;
        mysock.recv(response, zmq::recv_flags::none);
        //std::cout << response.to_string() << std::endl;
    }
    catch(zmq::error_t& exc) {
        std::cerr << "Could not send through ZMQ socket" << std::endl;
        std::cerr << exc.what() << std::endl;
        std::cerr << "(error num: " << exc.num() << ")" << std::endl;
        exit(-1);
    }
}

