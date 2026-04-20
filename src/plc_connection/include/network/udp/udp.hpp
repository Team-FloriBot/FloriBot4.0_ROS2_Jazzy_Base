#ifndef OWN_UDP_H
#define OWN_UDP_H

#include "network/socket/socket.hpp"

namespace OwnUDP
{
    #define checkAddress(x) OwnSocket::checkAddress(x)

    typedef OwnSocket::Address Address;
}

#endif