/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     serial.cpp
 * \author   Collin Johnson
 *
 * Definition of SerialConnection.
 */

#include "utils/serial.h"
#include <cstdio>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <linux/serial.h>   // high-speed serial support
#include <sys/select.h>
#include <unistd.h>


namespace vulcan
{
namespace utils
{

/** Create a connection to the specified serial port. */
int establish_serial_connection(const std::string& terminal,
                                Baud baudRate,
                                SerialMode mode,
                                bool useCanonical,
                                char eol,
                                struct termios& oldSettings);


Baud baud_from_int(int baud)
{
    Baud b = BAUD_0;

    switch (baud) {
    case 0:
        b = BAUD_0;
        break;
    case 50:
        b = BAUD_50;
        break;
    case 75:
        b = BAUD_75;
        break;
    case 110:
        b = BAUD_110;
        break;
    case 134:
        b = BAUD_134;
        break;
    case 150:
        b = BAUD_150;
        break;
    case 200:
        b = BAUD_200;
        break;
    case 300:
        b = BAUD_300;
        break;
    case 600:
        b = BAUD_600;
        break;
    case 1200:
        b = BAUD_1200;
        break;
    case 1800:
        b = BAUD_1800;
        break;
    case 2400:
        b = BAUD_2400;
        break;
    case 4800:
        b = BAUD_4800;
        break;
    case 9600:
        b = BAUD_9600;
        break;
    case 19200:
        b = BAUD_19200;
        break;
    case 38400:
        b = BAUD_38400;
        break;
    case 57600:
        b = BAUD_57600;
        break;
    case 115200:
        b = BAUD_115200;
        break;
    case 500000:
        b = BAUD_500000;
        break;
    default:
        b = BAUD_0;
    }

    return b;
}


SerialConnection::SerialConnection(const std::string& device,
                                   Baud baudRate,
                                   SerialMode mode,
                                   bool useCanonical,
                                   char eol)
: dev(device)
, baud(baudRate)
, parityMode(mode)
, useCanonical(useCanonical)
, eol(eol)
, fd(0)
{
    connect();
}


bool SerialConnection::connect(void)
{
    // Just establish the connection. That is all that is needed.
    fd = establish_serial_connection(dev, baud, parityMode, useCanonical, eol, oldSettings);

    return (fd != -1);
}


bool SerialConnection::write(const char* data, int dataLen)
{
    // If the data is null, then we're done, otherwise issue the write call

    if (fd == -1)   // invalid file descriptor, so can't write the data
    {
        return false;
    }

    // Null data?
    if (data == 0) {
        return true;
    }

    int success = ::write(fd, data, dataLen);

    return (success == dataLen);
}


bool SerialConnection::write(const unsigned char* data, int dataLen)
{
    // If the data is null, then we're done, otherwise issue the write call

    if (fd == -1)   // invalid file descriptor, so can't write the data
    {
        return false;
    }

    // Null data?
    if (data == 0) {
        return true;
    }

    int success = ::write(fd, data, dataLen);

    return (success == dataLen);
}


bool SerialConnection::write(const std::string& data)
{
    // If the data is null, then we're done, otherwise issue the write call

    if (fd == -1)   // invalid file descriptor, so cannot write a valid file
    {
        return false;
    }

    // Null data?
    if (data.size() == 0) {
        return true;
    }

    size_t success = ::write(fd, data.data(), data.size());

    return (success == data.size());
}


char* SerialConnection::read(char* buffer, int& bufLen)
{
    /*
     * Get the amount of data that needs to be read and then see if the buffer
     * is large enough. If so, read the data, otherwise, delete the old buffer
     * and allocate a new one before doing the read.
     */

    if (fd == -1)   // invalid file descriptor, bailout
    {
        bufLen = -1;
        return buffer;   // don't change the buffer in any way
    }

    int readLen = 0;

    if (bufLen <= 0) {
        ioctl(fd, FIONREAD, &readLen);

        if (readLen > bufLen) {
            if (buffer != 0) {
                delete buffer;
            }

            buffer = new char[readLen];

            bufLen = readLen;
        }
    } else {
        readLen = bufLen;
    }

    bufLen = ::read(fd, buffer, readLen);

    return buffer;
}


unsigned char* SerialConnection::read(unsigned char* buffer, int& bufLen)
{
    /*
     * Get the amount of data that needs to be read and then see if the buffer
     * is large enough. If so, read the data, otherwise, delete the old buffer
     * and allocate a new one before doing the read.
     */

    if (fd == -1)   // can't read from invalid file descriptor
    {
        bufLen = -1;
        return buffer;
    }

    // Doing the ioctl means that the read call will not block which is not the
    // desired behavior. Ignore this, attempt to read enough bytes to fill the provided
    // buffer with data

    // If, however, bufLen == 0, then we should make it large enough to hold the
    // data that is available, thus reallocation makes sense

    int readLen = 0;

    if (bufLen <= 0) {
        ioctl(fd, FIONREAD, &readLen);

        if (readLen > bufLen) {
            if (buffer != 0) {
                delete buffer;
            }

            buffer = new unsigned char[readLen];

            bufLen = readLen;
        }
    } else {
        readLen = bufLen;
    }

    bufLen = ::read(fd, buffer, readLen);

    return buffer;
}


void SerialConnection::flush(void)
{
    // To flush, just call a read with an empty pointer and read will read as much data from the buffer
    // as is there, then just delete this and all is well
    char* buffer = 0;
    int bufLen = 0;
    buffer = this->read(buffer, bufLen);

    if (bufLen) {
        std::cout << "Flushed: " << buffer << std::endl;
    }

    delete[] buffer;
}


bool SerialConnection::setBaudRate(Baud newBaud)
{
    // To change the baud rate, just snatch the termios and set the new baud.
    baud = newBaud;

    struct serial_struct
      reset;   // before doing any changes, reset the high-speed settings in case not going to that mode
    if (ioctl(fd, TIOCGSERIAL, &reset)) {
        std::cerr << "ERROR: Failed to get the custom serial settings." << std::endl;
        return false;
    }
    reset.flags &= ~ASYNC_SPD_CUST;
    reset.custom_divisor = 0;
    if (ioctl(fd, TIOCSSERIAL, &reset)) {
        std::cerr << "ERROR: Failed to set the custom serial settings." << std::endl;
        return false;
    }

    if (newBaud == BAUD_500000)   // for high-speed mode, a custom divisor needs to be set
    {
        struct serial_struct high;
        if (ioctl(fd, TIOCGSERIAL, &high)) {
            std::cerr << "ERROR: Failed to get settings for high-speed serial mode." << std::endl;
            return false;
        }

        high.flags |= ASYNC_SPD_CUST;
        high.custom_divisor = 48;   // for our particular chipset the divisor is 240/5 FTDI

        if (ioctl(fd, TIOCSSERIAL, &high)) {
            std::cerr << "ERROR: Failed to set settings for high-speed serial mode." << std::endl;
            return false;
        }

        // now that this has been done successfull, the way to get to high-speed mode is to set the
        // baud rate to 38400
        newBaud = BAUD_38400;
    }

    struct termios termOptions;
    // get the current attributes for the terminal
    if (tcgetattr(fd, &termOptions)) {
        std::cerr << "ERROR: Failed to get terminal options when changing baud rate." << std::endl;
        return false;
    }

    // now set the baudrate
    cfsetispeed(&termOptions, newBaud);
    cfsetospeed(&termOptions, newBaud);

    if (tcsetattr(fd, TCSANOW, &termOptions)) {
        std::cerr << "ERROR: Failed to set terminal options when changing baud rate." << std::endl;
        return false;
    }

    return true;
}


int SerialConnection::addToFDSet(fd_set& set) const
{
    FD_SET(fd, &set);

    return fd + 1;
}


bool SerialConnection::isReady(fd_set& set) const
{
    return FD_ISSET(fd, &set);
}


/**
 * establish_serial_connection establishes a connection to the terminal used for serial
 * port communications.
 *
 * \param    terminal            The terminal /dev/tty** the serial device is connected to
 * \param    baudRate            Baudrate of the connection to be established
 * \param    mode                Parity mode for the connection, i.e. 8N1, 7E1, or 8N2
 * \param    useCanonical        Flag indicating if the connection should be used in canonical mode
 * \param    eol                 End-of-line character to use if operating in canonical mode
 * \param    oldSettings         Previous settings for the serial port (output)
 * \return   Serial file descriptor needed to communicate with this serial device
 */
int establish_serial_connection(const std::string& terminal,
                                Baud baudRate,
                                SerialMode mode,
                                bool useCanonical,
                                char eol,
                                struct termios& oldSettings)
{
    /*
     * To establish a serial connection, the folliowing steps have to be performed:
     * 1) Open a connection to the terminal
     * 2) Get the termios structure associated with the terminal
     * 3) Set the baud rate of the connection
     * 4) Set the parity checking and data size (for now this will always be 8N1
     *    unless this ends up not being compartible with all our hardware
     * 5) Set the input mode
     * 6) Set the flow control mode (for all our sensors, this is 0)
     */

    // NOTE: Trying something slightly different here, s.t. parameters are set only a single time rather than with
    // multiple calls

    // open a connection to the desired terminal
    // options: O_RDWR - open for reading and writing
    //         0_NOCTTY - do not want to be the controlling terminal
    //         0_NDELAY - open regardless of what is on the other end of the terminal
    int serialFd = open(terminal.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    int n = fcntl(serialFd, F_GETFL, 0);
    fcntl(serialFd, F_SETFL, n & ~O_NDELAY);
    if (serialFd == -1)   // failed to open the port
    {
        std::cerr << "Error opening device: " << terminal << std::endl;
        perror("ERROR: Serial ");
        return serialFd;
    }

    // termios structure that holds all the information about the communication data
    struct termios termOptions;

    // get the current attributes for the terminal
    tcgetattr(serialFd, &termOptions);

    oldSettings = termOptions;   // Save the old so they can be reset when finished

    // now set the baudrate
    cfsetispeed(&termOptions, baudRate);
    cfsetospeed(&termOptions, baudRate);

    // set to be a receiver and local mode
    // CLOCAL - don't change the "owner" of the line, we're just peeking in on what is happening
    // CREAD - enable receiver, meaning it's time for us to start getting some data by saying that the driver should
    // start reading data
    termOptions.c_cflag |= (CLOCAL | CREAD);

    // set the options to the port
    // Using TCSANOW specifies that the changes should occur immediately
    // tcsetattr(serialFd, TCSANOW, &termOptions);

    // now establish the parity bits
    switch (mode) {
    case EIGHT_N_1:
        std::cout << "Setting 8N1 for serial line. " << std::endl;
        termOptions.c_cflag &= ~PARENB;   // no parity bit
        termOptions.c_cflag &= ~CSTOPB;   // use one stop bit
        termOptions.c_cflag &= ~CSIZE;    // negate the size
        termOptions.c_cflag |= CS8;       // set the size to be 8 bits
        break;

    case SEVEN_E_1:
        std::cout << "Setting 7E1 for serial line. " << std::endl;
        termOptions.c_cflag |= PARENB;   // use parity, defaults to even
        termOptions.c_cflag &= ~PARODD;
        termOptions.c_cflag &= ~CSTOPB;   // use one stop bit
        termOptions.c_cflag &= ~CSIZE;
        termOptions.c_cflag |= CS7;   // use 7 bits
        break;

    case EIGHT_N_2:
        std::cout << "Setting 8N2 for serial line." << std::endl;
        termOptions.c_cflag &= ~PARENB;   // no parity bit
        termOptions.c_cflag &= ~CSIZE;    // negate the size
        termOptions.c_cflag |= CSTOPB;    // use two stop bits instead of one
        termOptions.c_cflag |= CS8;       // set the size to be 8 bits
        break;

    default:
        std::cout << "Fuck it all I don't want to work" << std::endl;
    }

    // tcsetattr(serialFd, TCSANOW, &termOptions);

    // disable flow control  -- set in raw, no echo mode
    termOptions.c_cflag &= ~CRTSCTS;   // hardware flow control off
    termOptions.c_iflag = IGNBRK;
    termOptions.c_lflag = 0;
    termOptions.c_oflag = 0;
    termOptions.c_iflag &= ~(IXON | IXOFF | IXANY);   // software flow control

    // set to accept raw data rather than cannonical
    // due to the way data is handled in Sockets this will be the optimal choice
    if (!useCanonical) {
        termOptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    } else {
        termOptions.c_lflag |= ICANON;
        termOptions.c_cc[VEOL] = eol;
    }

    // now select raw output
    termOptions.c_oflag &= ~OPOST;

    termOptions.c_cflag |= HUPCL;   // just give this a try to see what happens with regards to the death of a
                                    // connection if terminated unexpectedly

    // now set all these new options to be put into effect immediately
    tcsetattr(serialFd, TCSANOW, &termOptions);

    std::cout << "Finished changing the settings for SerialConnection." << std::endl;

    return serialFd;
}

}   // namespace utils
}   // namespace vulcan
