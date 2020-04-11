#include "simplebot.hpp"

#include <cmath>
#include <iostream>
#include <boost/bind.hpp>

Simplebot::Simplebot(
        std::string port, 
        uint32_t baud,
        double axisLength,
        double turningAdaptation,
        double wheelRadius,
        int stepsPerRotation,
        double maxSpeed) : 
    m_io(), 
    m_serial(m_io, port),
    m_axisLength(axisLength),
    m_turningAdaptation(turningAdaptation),
    m_wheelRadius(wheelRadius),
    m_stepsPerRotation(stepsPerRotation),
    m_maxSpeed(maxSpeed),
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

bool checkParity(int64_t data) {
    int numOnes = 0;
    for (int i = 0; i < 64; i++) {
        if (data & (1 << i)) {
            numOnes++;
        }  
    }

    return numOnes % 2 == 0;
}

void Simplebot::setSpeed(double vL, double vR)
{
    int64_t rotL = vL * 10000 / (2 * M_PI * m_wheelRadius);
    int64_t rotR = vR * 10000 / (2 * M_PI * m_wheelRadius);

    uint8_t msg[19];
    
    msg[0] = 0xFF;

    msg[1] = (uint8_t)((rotL) & 0xFF);
    msg[2] = (uint8_t)((rotL >> 8) & 0xFF);
    msg[3] = (uint8_t)((rotL >> 16) & 0xFF);
    msg[4] = (uint8_t)((rotL >> 24) & 0xFF);
    msg[5] = (uint8_t)((rotL >> 32) & 0xFF);
    msg[6] = (uint8_t)((rotL >> 40) & 0xFF);
    msg[7] = (uint8_t)((rotL >> 48) & 0xFF);
    msg[8] = (uint8_t)((rotL >> 56) & 0xFF);

    msg[9] = (uint8_t)((rotR) & 0xFF);
    msg[10] = (uint8_t)((rotR >> 8) & 0xFF);
    msg[11] = (uint8_t)((rotR >> 16) & 0xFF);
    msg[12] = (uint8_t)((rotR >> 24) & 0xFF);
    msg[13] = (uint8_t)((rotR >> 32) & 0xFF);
    msg[14] = (uint8_t)((rotR >> 40) & 0xFF);
    msg[15] = (uint8_t)((rotR >> 48) & 0xFF);
    msg[16] = (uint8_t)((rotR >> 56) & 0xFF);

    msg[17] = (uint8_t) (!checkParity(rotL)) | (uint8_t) (!checkParity(rotR)) << 1;

    msg[18] = 0xFF;

    boost::asio::write(m_serial, boost::asio::buffer(msg, 19));
}

void Simplebot::odometryCallback(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error && bytes_transferred > 0)
    {
        if(m_receiveBuffer[0] == 0xFF)
        {
            long dRotL = 0;
            dRotL = dRotL
                    | (m_receiveBuffer[1])
                    | (((long) m_receiveBuffer[2]) << 8)
                    | (((long) m_receiveBuffer[3]) << 16)
                    | (((long) m_receiveBuffer[4]) << 24)
                    | (((long) m_receiveBuffer[5]) << 32)
                    | (((long) m_receiveBuffer[6]) << 40)
                    | (((long) m_receiveBuffer[7]) << 48)
                    | (((long) m_receiveBuffer[8]) << 56);

            long dRotR = 0;
            dRotR = dRotR
                    | (m_receiveBuffer[9])
                    | (((long) m_receiveBuffer[10]) << 8)
                    | (((long) m_receiveBuffer[11]) << 16)
                    | (((long) m_receiveBuffer[12]) << 24)
                    | (((long) m_receiveBuffer[13]) << 32)
                    | (((long) m_receiveBuffer[14]) << 40)
                    | (((long) m_receiveBuffer[15]) << 48)
                    | (((long) m_receiveBuffer[16]) << 56);

            double dRotLeft = (double) dRotL / 10000.0;
            double dRotRight = (double) dRotR / 10000.0;

            double distLeft = 2.0 * M_PI * m_wheelRadius * dRotLeft;
            double distRight = 2.0 * M_PI * m_wheelRadius * dRotRight;

            double dTheta = (distRight - distLeft) / m_axisLength * m_turningAdaptation;
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
            double vTheta = (distRight - distLeft) / m_axisLength * m_turningAdaptation;

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
