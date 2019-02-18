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
    bool isOpen();
    void closeConnection();
    bool open(std::string deviceName, int baud);
    bool numberByteRcv(int &bytelen);
    void sendMoveCommand(uint16_t vel, int16_t angle);
    void sendStopCommand();
    void sendTestCommand();
    void sendKickCommand();
    void sendChipCommand();
    void sendDribbleCommand(uint8_t dribbleSpeed);

};



#endif //TECHNICAL_CHALLENGE_2019_CONNECTION_H
