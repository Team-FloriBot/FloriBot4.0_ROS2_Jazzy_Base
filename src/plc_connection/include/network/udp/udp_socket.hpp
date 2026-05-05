#ifndef OWN_UDP_SOCKET_HPP
#define OWN_UDP_SOCKET_HPP

#include "network/udp/udp.hpp"

namespace OwnUDP
{
    class UDPSocket : private OwnSocket::Socket
    {
    public:
        UDPSocket();
        UDPSocket(Address* OwnIP);
        ~UDPSocket();

        void bindAddress(Address* OwnIP);
        void getAddress(Address* IP);

        void setReceiveTime(int usec, int sec);

        void write(uint8_t* Data, int length, Address* IP);
        void read(uint8_t* Data, int length, Address* IP);
    };
}

#endif
