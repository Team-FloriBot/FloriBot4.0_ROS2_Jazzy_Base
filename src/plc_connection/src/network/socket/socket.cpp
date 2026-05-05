#include "network/socket/socket.hpp"
#include <rclcpp/rclcpp.hpp>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <stdexcept>

// Standardkonstruktor
OwnSocket::Socket::Socket()
{
    SocketID_ = -1;
    Passive_ = false;
    Connected_ = false;
}

// Konstruktor mit Verbindungstyp
OwnSocket::Socket::Socket(Connection Type)
{
    SocketID_ = -1;
    Passive_ = false;
    Connected_ = false;

    init(Type);
}

// Destruktor
OwnSocket::Socket::~Socket()
{
    closeSocket();
}

// Initialisiere den Socket
void OwnSocket::Socket::init(Connection Type)
{
    struct sockaddr_in ownAddr{};
    socklen_t len = sizeof(ownAddr);

    if (SocketID_ >= 0)
    {
        closeSocket();
    }

    switch (Type)
    {
        case TCP:
            SocketID_ = socket(AF_INET, SOCK_STREAM, 0);
            break;

        case UDP:
            SocketID_ = socket(AF_INET, SOCK_DGRAM, 0);
            break;

        default:
            throw std::runtime_error("Invalid socket connection type");
    }

    if (SocketID_ < 0)
    {
        throw std::runtime_error("Socket is not valid: " + std::string(std::strerror(errno)));
    }

    if (getsockname(SocketID_, reinterpret_cast<sockaddr*>(&ownAddr), &len) < 0)
    {
        closeSocket();
        throw std::runtime_error("getsockname failed: " + std::string(std::strerror(errno)));
    }

    OwnAddress_.IP.clear();
    OwnAddress_.IP += inet_ntoa(ownAddr.sin_addr);
    OwnAddress_.Port = ntohs(ownAddr.sin_port);

    Type_ = Type;
}

// Setze Timeout für den Empfang von Daten
void OwnSocket::Socket::setReceiveTime(int usec, int sec)
{
    if (!socketOk())
    {
        throw std::runtime_error("Socket is not valid");
    }

    if (sec < 0 || usec < 0)
    {
        throw std::runtime_error("Invalid socket timeout");
    }

    struct timeval tv{};

    tv.tv_sec = sec + usec / 1000000;
    tv.tv_usec = usec % 1000000;

    if (setsockopt(
            SocketID_,
            SOL_SOCKET,
            SO_RCVTIMEO,
            reinterpret_cast<const char*>(&tv),
            sizeof(tv)) < 0)
    {
        throw std::runtime_error("Can not set Timeout: " + std::string(std::strerror(errno)));
    }
}

// Verbinde den Socket mit dem Ziel
void OwnSocket::Socket::connectTo(Address* IP)
{
    if (!socketOk())
    {
        throw std::runtime_error("Socket is not valid");
    }

    if (!checkAddress(IP))
    {
        throw std::runtime_error("Invalid Address parameter");
    }

    struct sockaddr_in server{};

    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(IP->IP.c_str());
    server.sin_port = htons(IP->Port);

    if (connect(SocketID_, reinterpret_cast<struct sockaddr*>(&server), sizeof(server)) < 0)
    {
        throw std::runtime_error(
            "Can not connect to IP " + IP->IP + ":" + std::to_string(IP->Port) +
            ": " + std::string(std::strerror(errno)));
    }

    Connected_ = true;
}

// Schließe den Socket
void OwnSocket::Socket::closeSocket()
{
    if (SocketID_ >= 0)
    {
        close(SocketID_);

        OwnAddress_.Port = 0;
        OwnAddress_.IP.clear();

        SocketID_ = -1;
        Connected_ = false;
        Passive_ = false;
    }
}

// Überprüfe, ob der Socket verbunden ist
bool OwnSocket::Socket::connected()
{
    return Connected_;
}

// Überprüfe, ob die Socket-ID gültig ist
bool OwnSocket::Socket::socketOk()
{
    return SocketID_ >= 0;
}

// Schreibe Daten
void OwnSocket::Socket::write(uint8_t* Data, int length, Address* Target)
{
    if (Data == nullptr)
    {
        throw std::runtime_error("Data pointer is null");
    }

    if (length <= 0)
    {
        throw std::runtime_error("Invalid write length");
    }

    if (Passive_)
    {
        throw std::runtime_error("Writing on passive Socket");
    }

    if (!socketOk())
    {
        throw std::runtime_error("Socket is not valid");
    }

    if (Connected_)
    {
        const ssize_t sent = send(SocketID_, Data, length, 0);

        if (sent < 0)
        {
            throw std::runtime_error("Sending Data failed: " + std::string(std::strerror(errno)));
        }

        if (sent != length)
        {
            throw std::runtime_error("Incomplete send");
        }

        return;
    }

    if (Type_ == TCP)
    {
        throw std::runtime_error("TCP Socket has to be connected to send Data");
    }

    if (Target == nullptr || !checkAddress(Target))
    {
        throw std::runtime_error("No valid IP-Address to send Data to");
    }

    struct sockaddr_in target{};

    target.sin_family = AF_INET;
    target.sin_addr.s_addr = inet_addr(Target->IP.c_str());
    target.sin_port = htons(Target->Port);

    const ssize_t sent = sendto(
        SocketID_,
        Data,
        length,
        0,
        reinterpret_cast<struct sockaddr*>(&target),
        sizeof(target));

    if (sent < 0)
    {
        std::stringstream strs;
        strs << "Error while sending message to "
             << Target->IP
             << ":"
             << Target->Port
             << ": "
             << std::strerror(errno);

        throw std::runtime_error(strs.str());
    }

    if (sent != length)
    {
        throw std::runtime_error("Incomplete UDP send");
    }
}

// Lese Daten vom Socket
void OwnSocket::Socket::read(uint8_t* Data, int length, Address* Source)
{
    if (Data == nullptr)
    {
        throw std::runtime_error("Data pointer is null");
    }

    if (length <= 0)
    {
        throw std::runtime_error("Invalid read length");
    }

    if (!socketOk())
    {
        throw std::runtime_error("Socket is not valid");
    }

    if (Type_ == TCP && !Connected_)
    {
        throw std::runtime_error("Socket has to be connected to receive Data");
    }

    struct sockaddr_in source{};
    socklen_t len = sizeof(source);

    const ssize_t received = recvfrom(
        SocketID_,
        Data,
        length,
        0,
        reinterpret_cast<struct sockaddr*>(&source),
        &len);

    if (received < 0)
    {
        throw std::runtime_error("Error while receiving Data: " + std::string(std::strerror(errno)));
    }

    if (received != length)
    {
        throw std::runtime_error(
            "Received packet has wrong size. Expected " +
            std::to_string(length) +
            " bytes, got " +
            std::to_string(received) +
            " bytes");
    }

    if (Source != nullptr)
    {
        Source->IP.clear();
        Source->IP += inet_ntoa(source.sin_addr);
        Source->Port = ntohs(source.sin_port);
    }
}

// Hole eigene Adresse
void OwnSocket::Socket::getAddress(Address* Target)
{
    if (Target == nullptr)
    {
        throw std::runtime_error("Target address pointer is null");
    }

    Target->IP.clear();
    Target->IP += OwnAddress_.IP;
    Target->Port = OwnAddress_.Port;
}

// Binde die angegebene IP-Adresse und den Port
void OwnSocket::Socket::bindAddress(Address* IPParam)
{
    if (!socketOk())
    {
        throw std::runtime_error("Socket is not valid");
    }

    if (!checkAddress(IPParam))
    {
        throw std::runtime_error(
            "Invalid IP Parameter Address: " +
            (IPParam != nullptr ? IPParam->IP : std::string("<null>")) +
            " Port: " +
            (IPParam != nullptr ? std::to_string(IPParam->Port) : std::string("<null>")));
    }

    struct sockaddr_in Addr{};

    Addr.sin_family = AF_INET;

    if (IPParam->IP.empty())
    {
        Addr.sin_addr.s_addr = INADDR_ANY;
    }
    else
    {
        Addr.sin_addr.s_addr = inet_addr(IPParam->IP.c_str());
    }

    Addr.sin_port = htons(IPParam->Port);

    int reuse = 1;
    setsockopt(SocketID_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    if (bind(SocketID_, reinterpret_cast<sockaddr*>(&Addr), sizeof(Addr)) < 0)
    {
        throw std::runtime_error(
            "Error while binding Address " +
            IPParam->IP +
            " with Port " +
            std::to_string(IPParam->Port) +
            ": " +
            std::string(std::strerror(errno)));
    }

    OwnAddress_.IP.clear();
    OwnAddress_.IP += IPParam->IP;
    OwnAddress_.Port = IPParam->Port;
}

// Akzeptiere eingehende Verbindungsanfragen
OwnSocket::Socket* OwnSocket::Socket::acceptConnection()
{
    if (!socketOk())
    {
        throw std::runtime_error("Socket is not valid");
    }

    struct sockaddr_in NewSock{};
    socklen_t len = sizeof(NewSock);

    const int newID = accept(SocketID_, reinterpret_cast<sockaddr*>(&NewSock), &len);

    if (newID < 0)
    {
        throw std::runtime_error("Socket can not accept connections: " + std::string(std::strerror(errno)));
    }

    return new Socket(newID, Type_);
}

// Setze Socket auf Lauschen
void OwnSocket::Socket::listenPort(int queuesSize)
{
    if (!socketOk())
    {
        throw std::runtime_error("Socket is not valid");
    }

    if (queuesSize <= 0)
    {
        throw std::runtime_error("Invalid listen queue size");
    }

    if (listen(SocketID_, queuesSize) < 0)
    {
        throw std::runtime_error("Socket can not listen: " + std::string(std::strerror(errno)));
    }

    Passive_ = true;
}

// Interner Konstruktor für akzeptierte Verbindungen
OwnSocket::Socket::Socket(int ID, Connection Type)
{
    struct sockaddr_in OwnSock{};
    socklen_t len = sizeof(OwnSock);

    SocketID_ = ID;
    Passive_ = false;
    Connected_ = true;
    Type_ = Type;

    if (SocketID_ < 0)
    {
        throw std::runtime_error("Invalid accepted socket ID");
    }

    if (getsockname(SocketID_, reinterpret_cast<sockaddr*>(&OwnSock), &len) < 0)
    {
        closeSocket();
        throw std::runtime_error("getsockname failed: " + std::string(std::strerror(errno)));
    }

    OwnAddress_.IP.clear();
    OwnAddress_.IP += inet_ntoa(OwnSock.sin_addr);
    OwnAddress_.Port = ntohs(OwnSock.sin_port);
}

// Gib den Verbindungstyp des Sockets zurück
OwnSocket::Connection OwnSocket::Socket::getConnectionType()
{
    return Type_;
}

// Überprüfe die IP-Adresse und die Portnummer
bool OwnSocket::checkAddress(Address* Addr)
{
    if (Addr == nullptr)
    {
        return false;
    }

    if (Addr->Port <= 0 || Addr->Port > 65535)
    {
        return false;
    }

    // Leere IP ist erlaubt und bedeutet INADDR_ANY beim Binden.
    if (Addr->IP.empty())
    {
        return true;
    }

    struct sockaddr_in tmp{};

    return inet_pton(AF_INET, Addr->IP.c_str(), &(tmp.sin_addr)) == 1;
}

// Ändere Float von Netzwerk- zu Host-Byte-Reihenfolge
float OwnSocket::ntohf(float In)
{
    uint32_t tmp = 0;
    float out = 0.0f;

    static_assert(sizeof(float) == sizeof(uint32_t), "Float must be 32 bit");

    std::memcpy(&tmp, &In, sizeof(float));
    tmp = ntohl(tmp);
    std::memcpy(&out, &tmp, sizeof(float));

    return out;
}

// Ändere Float von Host- zu Netzwerk-Byte-Reihenfolge
float OwnSocket::htonf(float In)
{
    uint32_t tmp = 0;
    float out = 0.0f;

    static_assert(sizeof(float) == sizeof(uint32_t), "Float must be 32 bit");

    std::memcpy(&tmp, &In, sizeof(float));
    tmp = htonl(tmp);
    std::memcpy(&out, &tmp, sizeof(float));

    return out;
}
