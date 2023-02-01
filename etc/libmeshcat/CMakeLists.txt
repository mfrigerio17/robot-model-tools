cmake_minimum_required(VERSION 3.0)
project(libmeshcat)


set(HEADER_INSTALL_ROOT /usr/local/include/ CACHE PATH "Where to install the headers")
set(LIB_INSTALL_ROOT    /usr/local/lib/     CACHE PATH "Where to install the shared library")

SET(CMAKE_CXX_FLAGS "-g -std=c++11 -Wall -O3")

set(LIB_NAME meshcatcpp)

set(INC_PATH include)
set(SRC_PATH src)

set(SOURCES
    ${SRC_PATH}/commands.cpp
    ${SRC_PATH}/zmqclient.cpp
    ${SRC_PATH}/buffer.cpp
)

include_directories(${INC_PATH})

# Add library
add_library(${LIB_NAME} SHARED ${SOURCES})

# Install (ie copy) header files
install(DIRECTORY ${INC_PATH}/meshcatcpp DESTINATION ${HEADER_INSTALL_ROOT})

# Install the shared library
install(TARGETS ${LIB_NAME} LIBRARY DESTINATION ${LIB_INSTALL_ROOT})
