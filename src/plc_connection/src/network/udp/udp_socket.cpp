#include "network/udp/udp_socket.hpp"

// Standardkonstruktor
OwnUDP::UDPSocket::UDPSocket()
{
    // Initialisiere den Socket und hole die zugewiesene Adresse
    OwnSocket::Socket::init(OwnSocket::Connection::UDP);
}

// Konstruktor mit angegebener Adresse
OwnUDP::UDPSocket::UDPSocket(Address* OwnIP)
{
    // Initialisiere den Socket und setze die IP-Adresse
    OwnSocket::Socket::init(OwnSocket::Connection::UDP);
    OwnUDP::UDPSocket::bindAddress(OwnIP);
}

// Weist die IP-Adresse zu
void OwnUDP::UDPSocket::bindAddress(Address* OwnIP)
{
    // Binde die Adresse
    OwnSocket::Socket::bindAddress(OwnIP);
}

// Sendet Daten
void OwnUDP::UDPSocket::write(uint8_t* Data, int length, Address* IP)
{
    OwnSocket::Socket::write(Data, length, IP);   
}

// Liest Daten
void OwnUDP::UDPSocket::read(uint8_t* Data, int length, Address* IP)
{
    OwnSocket::Socket::read(Data, length, IP);
}

// Gibt die eigene IP-Adresse zurück
void OwnUDP::UDPSocket::getAddress(Address* IP)
{
    IP->IP.clear();
    IP->IP += OwnAddress_.IP;
    IP->Port = OwnAddress_.Port;
}

// Setzt die Empfangszeit
void OwnUDP::UDPSocket::setReceiveTime(int usec, int sec)
{
    OwnSocket::Socket::setReceiveTime(usec, sec);
}

// Destruktor
OwnUDP::UDPSocket::~UDPSocket()
{}