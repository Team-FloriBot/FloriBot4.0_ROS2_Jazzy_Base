#include "network/socket/socket.hpp"
#include <rclcpp/rclcpp.hpp>

// Standardkonstruktor
OwnSocket::Socket::Socket()
{   
    // Initialisiere den Socket
    SocketID_ = -1;
    Passive_ = false;
    Connected_ = false;
}

// Konstruktor mit Verbindungstyp
OwnSocket::Socket::Socket(Connection Type)
{
    // Initialisiere Werte
    SocketID_ = -1;
    Passive_ = false;
    Connected_ = false;

    // Initialisiere den Socket
    init(Type);
}

// Destruktor
OwnSocket::Socket::~Socket()
{
    // Schließe den Socket vor dem Löschen des Objekts
    closeSocket();
}

// Initialisiere den Socket
void OwnSocket::Socket::init(Connection Type)
{
    // Erstelle temporäre Variablen
    struct sockaddr_in ownAddr;
    socklen_t len;

    // Überprüfe, ob der Socket bereits initialisiert ist
    if (SocketID_ > 0) closeSocket();

    // Initialisiere den Socket je nach Verbindungstyp
    switch (Type)
    {
        case TCP:
            SocketID_ = socket(AF_INET, SOCK_STREAM, 0);
            break;
        case UDP:
            SocketID_ = socket(AF_INET, SOCK_DGRAM, 0);
            break;
    }

    // Überprüfe, ob der Socket erfolgreich erstellt wurde
    if (SocketID_ <= 0) throw std::runtime_error("Socket is not valid");

    // Hole Socket-Daten vom System
    getsockname(SocketID_, (sockaddr*)&ownAddr, &len);

    // Speichere Socket-Daten
    OwnAddress_.IP.clear();
    OwnAddress_.IP += inet_ntoa(ownAddr.sin_addr);
    OwnAddress_.Port = ntohs(ownAddr.sin_port);

    // Speichere den Socket-Typ
    Type_ = Type;
}

// Setze Timeout für den Empfang von Daten
void OwnSocket::Socket::setReceiveTime(int usec, int sec)
{
    // Erstelle temporäre Variablen
    struct timeval tv;

    // Setze die Zeitvariable
    tv.tv_sec = sec;       
    tv.tv_usec = usec; 
    // Setze Timeout für den Socket     
    if (setsockopt(SocketID_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval)) < 0) throw std::runtime_error("Can not set Timeout");
}

// Verbinde den Socket mit dem Ziel
void OwnSocket::Socket::connectTo(Address* IP)
{
    // Erstelle temporäre Variablen
    struct sockaddr_in server;

    if (!checkAddress(IP)) throw std::runtime_error("Invalid Address parameter");
    // Setze Server-Verbindungsdaten
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(IP->IP.c_str());
    server.sin_port = htons(IP->Port);

    // Verbinde zum Server
    if (connect(SocketID_, (struct sockaddr *) &server, sizeof(server) < 0)) throw std::runtime_error("Can not Connect to IP " + IP->IP);
    else Connected_ = true;
}

// Schließe den Socket
void OwnSocket::Socket::closeSocket()
{
    // Überprüfe, ob der Socket verbunden ist
    if (SocketID_ > 0)
    {  
        // Schließe den Socket
        close(SocketID_);

        // Setze Socket-Daten zurück
        OwnAddress_.Port = 0;
        OwnAddress_.IP.clear();
        SocketID_ = 0;
        Connected_ = false;
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
    return (SocketID_ > 0);
}

// Schreibe Daten
void OwnSocket::Socket::write(uint8_t* Data, int length, Address* Target = NULL)
{
    // Überprüfe den Socket-Status  
    if (Passive_) throw std::runtime_error("Writing on passive Socket");
    if (!socketOk()) throw std::runtime_error("Socket is not valid");

    // Überprüfe, ob verbunden
    if (Connected_)
    {
        // Sende Daten
        if (send(SocketID_, Data, length, 0) < 0) throw std::runtime_error("Sending Data failed");
    }
    else
    {
        // Überprüfe, ob eine Verbindung erforderlich ist
        if (Type_ == TCP) throw std::runtime_error("TCP Socket has to be connected to send Data");

        // Überprüfe die Adresse zum Senden
        if (Target->IP.empty() || !checkAddress(Target)) throw std::runtime_error("No valid IP-Address to send Data to");

        struct sockaddr_in target;

        // Setze Daten für das Ziel
        target.sin_family = AF_INET;
        target.sin_addr.s_addr = inet_addr(Target->IP.c_str());
        target.sin_port = htons(Target->Port);

        // Sende Daten
        int s = sendto(SocketID_, Data, length, 0, (struct sockaddr*)&target, sizeof(target));

        if (s < 0) 
        {
            std::stringstream strs;
            strs << "Error while sending message to " << Target->IP << ":" << Target->Port;
            throw std::runtime_error(strs.str());
        }
    }   
}

// Lese Daten vom Socket
void OwnSocket::Socket::read(uint8_t* Data, int length, Address* Source = NULL)
{
    // Überprüfe den Socket
    if (!socketOk()) throw std::runtime_error("Socket is not valid");

    // Überprüfe, ob eine aktive Verbindung erforderlich ist, um Nachrichten zu empfangen
    if (Type_ == TCP && !Connected_) throw std::runtime_error("Socket has to be connected to receive Data");

    socklen_t len;
    struct sockaddr_in source;

    // Empfange Daten und erhalte die Quelladresse
    if (recvfrom(SocketID_, Data, length, 0, (struct sockaddr*)&source, &len) < 0) throw std::runtime_error("Error while receiving Data"); 

    // Schreibe die Quelladressdaten für die weitere Verwendung
    if (Source != NULL)
    {
        Source->IP.clear();
        Source->IP += inet_ntoa(source.sin_addr);
        Source->Port = ntohs(source.sin_port);
    }
}

// Hole eigene Adresse 
void OwnSocket::Socket::getAddress(Address* Target)
{
    Target->IP.clear();
    Target->IP += OwnAddress_.IP;
    Target->Port = OwnAddress_.Port;
}

// Binde die angegebene IP-Adresse und den Port
void OwnSocket::Socket::bindAddress(Address* IPParam)
{
    struct sockaddr_in Addr;

    if (!checkAddress(IPParam)) throw std::runtime_error("Invalid IP Parameter Address: " + IPParam->IP + " Port: " + std::to_string(IPParam->Port));

    // Bereite Daten vor, um die Adressdaten zu setzen
    Addr.sin_family = AF_INET;

    if (IPParam->IP.empty())
        Addr.sin_addr.s_addr = INADDR_ANY;
    else
        Addr.sin_addr.s_addr = inet_addr(IPParam->IP.c_str());
    
    Addr.sin_port = htons(IPParam->Port);

    // Binde die Adresse an den Socket
    if (bind(SocketID_, (sockaddr*)&Addr, sizeof(Addr)) < 0) 
                throw std::runtime_error("Error while binding Address " + IPParam->IP + " with Port " + std::to_string(IPParam->Port));
    else
    {
        // Ändere gespeicherte Socket-Daten
        OwnAddress_.IP.clear();
        OwnAddress_.IP += IPParam->IP;
        OwnAddress_.Port = IPParam->Port;
    }   
}

// Akzeptiere eingehende Verbindungsanfragen
OwnSocket::Socket* OwnSocket::Socket::acceptConnection()
{
    // Erstelle temporäre Variablen
    struct sockaddr_in NewSock;
    socklen_t len = sizeof(NewSock);
    int newID;

    // Akzeptiere Verbindung
    if (newID = accept(SocketID_, (sockaddr*)&NewSock, &len) < 0) throw std::runtime_error("Socket can not accept connections");

    // Gib ein Socket-Objekt für die neue Verbindung zurück
    return new Socket(newID, Type_);
}

// Setze Socket auf Lauschen
void OwnSocket::Socket::listenPort(int queuesSize)
{
    // Setze den Socket
    if (listen(SocketID_, queuesSize) < 0) throw std::runtime_error("Socket can not listen");
    Passive_ = true;
}

// Interner Konstruktor für akzeptierte Verbindungen
OwnSocket::Socket::Socket(int ID, Connection Type)
{
    // Temporäre Variablen
    struct sockaddr_in OwnSock;
    socklen_t len;

    // Schreibe Daten in das Objekt
    SocketID_ = ID;
    getsockname(SocketID_, (sockaddr*)&OwnSock, &len);

    OwnAddress_.IP.clear();
    OwnAddress_.IP += inet_ntoa(OwnSock.sin_addr);
    OwnAddress_.Port = ntohs(OwnSock.sin_port);
    Connected_ = true;
    Type_ = Type;
}

// Gib den Verbindungstyp des Sockets zurück
OwnSocket::Connection OwnSocket::Socket::getConnectionType()
{
    return Type_;
}

// Überprüfe die IP-Adresse und die Portnummer, gibt true zurück, wenn die Adresse in Ordnung ist
bool OwnSocket::checkAddress(Address* Addr)
{
    // Initialisiere Regex zum Überprüfen der IP-Adresse
    std::regex regtest("^(?:[0-9]{1,3}[.]){3}[0-9]{1,3}$");
    // Überprüfe die Adressdaten
    if (!std::regex_match(Addr->IP.begin(), Addr->IP.end(), regtest) && Addr->IP.empty()) return false;
    if (Addr->Port <= 0) return false;

    return true;
}

// Ändere Float von Host- zu Netzwerk-Byte-Reihenfolge !!Funktioniert nur für 32-Bit-Float-Typ!!
float OwnSocket::ntohf(float In)
{
    // Erstelle temporäre Variablen
    uint32_t tmp;
    float out;

    // Kopiere Float zu Int, ändere die Byte-Reihenfolge und kopiere zurück
    memcpy(&tmp, &In, sizeof(float));
    tmp = ntohl(tmp);
    memcpy(&out, &tmp, sizeof(float));
    return out;
}

// Ändere Float von Netzwerk- zu Host-Byte-Reihenfolge !!Funktioniert nur für 32-Bit-Float-Typ!!
float OwnSocket::htonf(float In)
{
    // Erstelle temporäre Variablen
    uint32_t tmp;
    float out;

    // Ändere die Byte-Reihenfolge
    memcpy(&tmp, &In, sizeof(float));
    tmp = ntohl(tmp);
    memcpy(&out, &tmp, sizeof(float));
    return out;
}