#ifndef SIMPLEBOT_HPP
#define SIMPLEBOT_HPP

#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

class Simplebot 
{
public:
    Simplebot(std::string port, uint32_t baud, double wheelRadius = 0.025, int stepsPerRotation = 200, double maxSpeed = 1);
    ~Simplebot() {}
    
    void setSpeed(double vL, double vR);
    
    void odometryCallback(const boost::system::error_code& error, std::size_t bytes_transferred);

    void setOnOdometryHandler(std::function<void(double, double, double, double, double, double, double)>& onOdometryReceived);

    double m_maxSpeed;

private:
    void readOdometry();

    boost::thread m_ioThread;
    boost::asio::io_service m_io;
    boost::asio::serial_port m_serial;

    std::function<void(double, double, double, double, double, double, double)> m_onOdometryReceived;

    double m_lastRotLeft;
    double m_lastRotRight;

    double m_wheelRadius;
    int m_stepsPerRotation;

    double m_x;
    double m_y;

    double m_theta;

    uint8_t m_receiveBuffer[17];
};

#endif
