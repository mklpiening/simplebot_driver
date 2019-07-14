#include "simplebot.hpp"

#include <iostream>

Simplebot::Simplebot(
        std::string port, 
        uint32_t baud, 
        double maxSpeed) : 
    m_io(), 
    m_serial(m_io, port), 
    m_maxSpeed(maxSpeed) 
{
    m_serial.set_option(boost::asio::serial_port_base::baud_rate(baud));
}

void Simplebot::setSpeed(double vL, double vR) 
{
    uint8_t msg[4];
    
    msg[0] = 0xFF;
    msg[1] = (vL < 0 ? 2 : 0) | (vR < 0 ? 1 : 0);
    msg[2] = 0xFF * ((vL < 0 ? -1 : 1) * vL);
    msg[3] = 0xFF * ((vR < 0 ? -1 : 1) * vR);

    boost::asio::write(m_serial, boost::asio::buffer(msg, 4));
}