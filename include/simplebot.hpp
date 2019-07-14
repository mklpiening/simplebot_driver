#ifndef SIMPLEBOT_HPP
#define SIMPLEBOT_HPP

#include <string>
#include <boost/asio.hpp>

class Simplebot 
{
public:
    Simplebot(std::string port, uint32_t baud, double maxSpeed = 1);
    ~Simplebot() {}
    
    void setSpeed(double vL, double vR);
    
    double m_maxSpeed;

private:
    boost::asio::io_service m_io;
    boost::asio::serial_port m_serial;
};

#endif