extern "C" {
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
}

#include <iostream>
#include <fstream>


#include "Connection.h"

Connection::Connection(std::string deviceName) {
    handle = -1;
    openConnection(deviceName);
    lastVelocity = 0;
    lastAngle = 0;


}

Connection::~Connection() {
    if (handle >= 0)
        closeConnection();
}

void Connection::closeConnection() {
    if (handle >= 0) {
        close(handle);
    }
    handle = -1;
}


bool Connection::openConnection(std::string deviceName) {
    struct termios tio;
    struct termios2 tio2;
    this->deviceName = deviceName;
    this->baud = 115200;
//    this->fileIO = new ofstream(deviceName)


    handle = open(this->deviceName.c_str(), O_RDWR | O_NOCTTY  | O_NONBLOCK );

    if (handle < 0)
        return false;
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_oflag = 0;
    tio.c_lflag = 0;       //ICANON;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;     // time out every .1 sec
    ioctl(handle, TCSETS, &tio);

    ioctl(handle, TCGETS2, &tio2);
    tio2.c_cflag &= ~CBAUD;
    tio2.c_cflag |= BOTHER;
    tio2.c_ispeed = static_cast<speed_t>(baud);
    tio2.c_ospeed = static_cast<speed_t>(baud);
    ioctl(handle, TCSETS2, &tio2);

//   flush buffer
    ioctl(handle, TCFLSH, TCIOFLUSH);

    return true;
}

bool Connection::isOpen() {
    return (handle >= 0);
}

bool Connection::sendOverConnection(unsigned char *data, int len) {
    if (!isOpen()) return false;
    int rlen = write(handle, data, len);
    return (rlen == len);
}

bool Connection::sendOverConnection(unsigned char value) {
    if (!isOpen()) return false;
    int rlen = write(handle, &value, 1);
    return (rlen == 1);
}


bool Connection::sendOverConnection(std::string value) {
    if (!isOpen()) return false;
    int rlen = static_cast<int>(write(handle, value.c_str(), value.size()));
    return (rlen == value.size());

//    (*outFile) << value;
}


int Connection::receiveOverConnection(unsigned char *data, int len) {
    if (!isOpen()) return -1;

    // this is a blocking receives
    int lenRCV = 0;
    while (lenRCV < len) {
        int rlen = read(handle, &data[lenRCV], len - lenRCV);
        lenRCV += rlen;
    }
    return lenRCV;
}

bool Connection::numberByteRcv(int &bytelen) {
    if (!isOpen()) return false;
    ioctl(handle, FIONREAD, &bytelen);
    return true;
}

void Connection::sendMoveCommand(uint16_t vel, int16_t angle) {

    lastVelocity = vel;
    lastAngle = angle;

    // TODO parse 2 ints to one 32 bit int, check if this is correct
    uint32_t parsed_int = (angle << 16) | (vel);


    std::string to_send = std::to_string(parsed_int);
//    usleep(1000);


    sendOverConnection("vel_command " + to_send + "\r");
    usleep(1000);
//    sendOverConnection("\r\n");
//    sendOverConnection("\r\n");

}

void Connection::sendStopCommand() {
    sendMoveCommand(0, 0);
}

void Connection::sendTestCommand() {

    uint16_t vel = 1000;
    int16_t angle = 0;

    sendMoveCommand(vel, angle);

    usleep(100000);

    sendStopCommand();


}

void Connection::sendKickCommand() {
    sendOverConnection("kick 99\r");
}

void Connection::sendChipCommand() {
    sendOverConnection("chip\r");
}

void Connection::sendDribbleCommand(uint8_t dribbleSpeed) {
    sendOverConnection("dribble " + std::to_string(dribbleSpeed) + "\r");
}


//huart3.Instance = USART3;
//huart3.Init.BaudRate = 115200;
//huart3.Init.WordLength = UART_WORDLENGTH_8B;
//huart3.Init.StopBits = UART_STOPBITS_1;
//huart3.Init.Parity = UART_PARITY_NONE;
//huart3.Init.Mode = UART_MODE_TX_RX;
//huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//huart3.Init.OverSampling = UART_OVERSAMPLING_16;