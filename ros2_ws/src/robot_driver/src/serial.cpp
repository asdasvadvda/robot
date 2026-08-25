#include "robot_driver/serial.hpp"

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>


SerialPort::SerialPort()
{
    fd_ = -1;
}


SerialPort::~SerialPort()
{
    closePort();
}



bool SerialPort::openPort(
    const std::string &port,
    int baudrate
)
{

    fd_ = open(
        port.c_str(),
        O_RDWR | O_NOCTTY
    );


    if(fd_ < 0)
    {
        std::cerr 
        << "Cannot open serial port: "
        << port
        << std::endl;

        return false;
    }


    struct termios tty{};


    tcgetattr(fd_, &tty);


    cfsetospeed(
        &tty,
        B115200
    );

    cfsetispeed(
        &tty,
        B115200
    );


    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;


    tty.c_cflag &= ~PARENB;

    tty.c_cflag &= ~CSTOPB;


    tty.c_lflag = 0;

    tty.c_oflag = 0;


    tcsetattr(
        fd_,
        TCSANOW,
        &tty
    );


    return true;

}



void SerialPort::closePort()
{

    if(fd_ >= 0)
    {
        close(fd_);
        fd_ = -1;
    }

}



bool SerialPort::writeData(
    const std::string &data
)
{

    if(fd_ < 0)
        return false;


    write(
        fd_,
        data.c_str(),
        data.size()
    );


    return true;

}



std::string SerialPort::readData()
{

    char buffer[256];

    int n =
    read(
        fd_,
        buffer,
        sizeof(buffer)
    );


    if(n > 0)
    {
        return std::string(
            buffer,
            n
        );
    }


    return "";

}
