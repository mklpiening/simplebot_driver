#include "simplebot.hpp"

#include <cmath>
#include <iostream>
#include <boost/bind.hpp>

Simplebot::Simplebot(
        std::string port, 
        uint32_t baud,
        double axisLength,
        double wheelRadius,
        int stepsPerRotation,
        double maxSpeed) : 
    m_io(), 
    m_serial(m_io, port),
    m_axisLength(axisLength),
    m_wheelRadius(wheelRadius),
    m_stepsPerRotation(stepsPerRotation),
    m_maxSpeed(maxSpeed),
    m_lastRotLeft(0.0),
    m_lastRotRight(0.0),
    m_x(0.0),
    m_y(0.0),
    m_theta(0.0),
    m_onOdometryReceived(nullptr)
{
    m_serial.set_option(boost::asio::serial_port_base::baud_rate(baud));

    // wait for connection
    sleep(2);

    readOdometry();

    boost::thread t(boost::bind(&boost::asio::io_service::run, &m_io));
    m_ioThread.swap(t);
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

void Simplebot::odometryCallback(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error && bytes_transferred > 0)
    {
        if(m_receiveBuffer[0] == 0xFF)
        {
            long stepsLeft = 0;
            stepsLeft = stepsLeft
                    | (m_receiveBuffer[1])
                    | (((long) m_receiveBuffer[2]) << 8)
                    | (((long) m_receiveBuffer[3]) << 16)
                    | (((long) m_receiveBuffer[4]) << 24)
                    | (((long) m_receiveBuffer[5]) << 32)
                    | (((long) m_receiveBuffer[6]) << 40)
                    | (((long) m_receiveBuffer[7]) << 48)
                    | (((long) m_receiveBuffer[8]) << 56);

            long stepsRight = 0;
            stepsRight = stepsRight
                    | (m_receiveBuffer[9])
                    | (((long) m_receiveBuffer[10]) << 8)
                    | (((long) m_receiveBuffer[11]) << 16)
                    | (((long) m_receiveBuffer[12]) << 24)
                    | (((long) m_receiveBuffer[13]) << 32)
                    | (((long) m_receiveBuffer[14]) << 40)
                    | (((long) m_receiveBuffer[15]) << 48)
                    | (((long) m_receiveBuffer[16]) << 56);

            double rotLeft = (double) stepsLeft / (double) m_stepsPerRotation;
            double rotRight = (double) stepsRight / (double) m_stepsPerRotation;

            double dRotLeft = rotLeft - m_lastRotLeft;
            double dRotRight = rotRight - m_lastRotRight;

            m_lastRotLeft = rotLeft;
            m_lastRotRight = rotRight;


            double distLeft = 2.0 * M_PI * m_wheelRadius * dRotLeft;
            double distRight = 2.0 * M_PI * m_wheelRadius * dRotRight;

            double dTheta = (distRight - distLeft) / 0.18;
            double hypothenuse = 0.5 * (distLeft + distRight);

            if (abs(hypothenuse * cos(m_theta + dTheta * 0.5)) < 1) {
                m_x += hypothenuse * cos(m_theta + dTheta * 0.5);
                m_y += hypothenuse * sin(m_theta + dTheta * 0.5);

                m_theta += dTheta;
                if (m_theta > M_PI)
                {
                    m_theta -= 2.0 * M_PI;
                }
                if (m_theta < -M_PI)
                {
                    m_theta += 2.0 * M_PI;
                }
            }

            double wheelsLeft = 2.0 * M_PI * fmod(dRotLeft, 1);
            if (wheelsLeft > M_PI)
            {
                wheelsLeft -= 2.0 * M_PI;
            }
            if (wheelsLeft < -M_PI)
            {
                wheelsLeft += 2.0 * M_PI;
            }

            double wheelsRight = 2.0 * M_PI * fmod(dRotRight, 1);
            if (wheelsRight > M_PI)
            {
                wheelsRight -= 2.0 * M_PI;
            }
            if (wheelsRight < -M_PI)
            {
                wheelsRight += 2.0 * M_PI;
            }

            double vX = (distLeft + distRight) * 0.5;
            double vTheta = (distRight - distLeft) / m_axisLength;

            if (m_onOdometryReceived)
            {
                m_onOdometryReceived(m_x, m_y, m_theta, vX, vTheta, wheelsLeft, wheelsRight);
            }
        }
    }

    readOdometry();
}

void Simplebot::readOdometry()
{
    m_serial.async_read_some(boost::asio::buffer(m_receiveBuffer, 17),
                boost::bind(
                    &Simplebot::odometryCallback,
                    this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}

void Simplebot::setOnOdometryHandler(std::function<void(double, double, double, double, double, double, double)>& onOdometryReceived)
{
    m_onOdometryReceived = onOdometryReceived;
}
