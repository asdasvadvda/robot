#ifndef ROBOT_DRIVER_SERIAL_HPP
#define ROBOT_DRIVER_SERIAL_HPP

#include <string>

class SerialPort
{
public:

    SerialPort();

    ~SerialPort();

    bool openPort(
        const std::string &port,
        int baudrate
    );

    void closePort();

    bool writeData(
        const std::string &data
    );

    std::string readData();


private:

    int fd_;   //保存当前打开的串口编号

};

#endif
