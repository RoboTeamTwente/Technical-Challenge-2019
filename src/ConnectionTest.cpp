#include "Connection.cpp"
#include <iostream>
using namespace std;



int  main(void)
{

    Connection serial("/dev/ttyAMA0",115200);

    // One Byte At the time


    // An array of byte
    unsigned char  dataArray[] = { 142,0};
    serial.Send(dataArray,sizeof(dataArray));

    // Or a string
    serial.Send("forward\r\n");

    return 0;
}