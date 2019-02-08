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

    bool Send( unsigned char  * data,int len);
    bool Send(unsigned char value);
    bool Send( std::string value);
    int Receive( unsigned char  * data, int len);
    bool IsOpen(void);
    void Close(void);
    bool Open(std::string deviceName, int baud);
    bool NumberByteRcv(int &bytelen);
};



#endif //TECHNICAL_CHALLENGE_2019_CONNECTION_H
