//
// Created by freek on 08/02/19.
//

#ifndef TECHNICAL_CHALLENGE_2019_CONNECTION_H
#define TECHNICAL_CHALLENGE_2019_CONNECTION_H

#include <string>


class  Connection
{

public:

    int handle;
    std::string  deviceName;
    int baud;

    Connection(std::string deviceName, int baud);
    ~Connection();

    bool send( unsigned char  * data,int len);
    bool send(unsigned char value);
    bool send( std::string value);
    int receive( unsigned char  * data, int len);
    bool isOpen(void);
    void close(void);
    bool open(std::string deviceName, int baud);
    bool numberByteRcv(int &bytelen);
    void sendCommand();
};



#endif //TECHNICAL_CHALLENGE_2019_CONNECTION_H
