/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     serial.h
* \author   Collin Johnson
*
* Declaration of SerialConnection for talking over a serial port.
*/

#ifndef UTILS_SERIAL_H
#define UTILS_SERIAL_H

#include <sys/ioctl.h>
#include <termios.h>   //for serial port stuff
#include <unistd.h>
#include <string>
#include <unistd.h>

namespace vulcan
{
namespace utils
{

/** Baud specifies the possible baud rates for serial communications. */
enum Baud
{
    BAUD_0 = B0,
    BAUD_50 = B50,
    BAUD_75 = B75,
    BAUD_110 = B110,
    BAUD_134 = B134,
    BAUD_150 = B150,
    BAUD_200 = B200,
    BAUD_300 = B300,
    BAUD_600 = B600,
    BAUD_1200 = B1200,
    BAUD_1800 = B1800,
    BAUD_2400 = B2400,
    BAUD_4800 = B4800,
    BAUD_9600 = B9600,
    BAUD_19200 = B19200,
    BAUD_38400 = B38400,
    BAUD_57600 = B57600,
    BAUD_115200 = B115200,
    BAUD_500000
};

/**
* baud_from_int finds the baud rate associated with a given integer.
*
* \param    baud            Integer value representing the baud
* \return   Baud enum value associated with the given integer. If no such value exists, the Baud returned is BAUD_0.
*/
Baud baud_from_int(int baud);

/** SerialMode specifies the mode for the connection. */
enum SerialMode
{
    EIGHT_N_1,          ///< 8N1 operation -- 8 data bits, no parity,   1 stop bit
    EIGHT_N_2,          ///< 8N2 operation -- 8 data bits, no parity,   2 stop bits
    SEVEN_E_1           ///< 7E1 operation -- 7 data bits, even parity, 1 stop bit
};

/**
* SerialConnection is a class that establishes and maintains a connection to a
* serial port. Serial connection is a very bare interface for the serial line,
* abstracting away the lowest levels details of communicating with the serial port.
*
* SerialConnection is intended for synchronous communication. As a result, calls
* to readData() will block until data arrives. If asynchronous behavior is desired,
* the Sockets class should be used.
*
* All connections established by the SerialConnection class use 8N1 settings with
* no flow control because all the sensors we use have those settings. No need to
* try and get fancy as as result.
*/
class SerialConnection
{
public:

    /**
    * Constructor for SerialConnection.
    *
    * After successful construction, the SerialConnection will be ready for read/write.
    * connect() does not need to be called.
    *
    * \param    device              Serial device to connect to 'dev/tty**'
    * \param    baudRate            Baud rate of the connection
    * \param    mode                Mode of operation for the connection
    * \param    useCanonical        Use canonical mode
    * \param    eol                 End-of-line for canonical mode
    */
    SerialConnection(const std::string& device, Baud baudRate, SerialMode mode = EIGHT_N_1, bool useCanonical = false, char eol = 0);

    /**
    * Destructor for SerialConnection. Closes the connection
    */
    ~SerialConnection(void)
    {
        tcsetattr(fd, TCSANOW, &oldSettings);
        this->close();
    }

    /**
    * connect establishes the link to the serial device. After a successful call
    * to connect(), data transmission can begin on the connection.
    *
    * \return   True if the connection is successfully established
    */
    bool connect(void);

    /** close closes the connection. */
    void close(void) { ::close(fd); }

    /**
    * write writes data to the connection. The call will block until all the data
    * has been written.
    *
    * \param    data                Data to write
    * \param    dataLen             Length of the data
    * \return   True if the write is successful
    */
    bool write(const char* data, int dataLen);
    
    /**
    * write writes data to the connection. The call will block until all the data
    * has been written.
    *
    * \param    data                Data to write
    * \param    dataLen             Length of the data
    * \return   True if the write is successful
    */
    bool write(const unsigned char* data, int dataLen);

    /**
    * write writes data to the connection. The call will block until all the data
    * has been written.
    *
    * \param    data                Data to write
    * \return   True if the write is successful
    */
    bool write(const std::string& data);

    /**
    * read reads data from the connection. The supplied buffer will be resized according
    * to the amount of data that needs to be written. The dataLen parameter specifies
    * the current length of the buffer that is supplied. If the read() function needs
    * to reallocate the buffer to make it large enough to store all the data, the
    * bufLen parameter will be updated.
    *
    * \param    buffer              Buffer to read data into
    * \param    bufLen              Length of the buffer
    * \return   Pointer to the buffer the data was read into.
    */
    char*          read(char*          buffer, int& len);
    unsigned char* read(unsigned char* buffer, int& len);

    /**
    * flush clears all data from the serial line buffer. Any data the was on the line is immediately
    * discarded and cannot be retrieved.
    */
    void flush(void);

    /**
    * getBaudRate retrieves the current baud rate of the SerialConnection.
    *
    * \return   Baud of the connection.
    */
    Baud getBaudRate(void) const { return baud; }

    /**
    * setBaudRate changes the baud rate of connection. If the change fails, the
    * connection will no longer valid. Just a warning.
    *
    * \param    newBaud             New Baud to set for the connection
    * \return   True if the baud rate is successfully changed.
    */
    bool setBaudRate(Baud newBaud);

    /**
    * getDevice retrieves the device this SerialConnection is communicating with.
    *
    * \return   Device represented by the SerialConnection.
    */
    std::string getDevice(void) const { return dev; }

    /**
    * addToFDSet adds this connection to an fd_set for use in a select statement.
    *
    * \param    set         Set to add connection to
    * \return   Minimum value to use for nfds with this connection.
    */
    int addToFDSet(fd_set& set) const;

    /**
    * isReady checks if this connection is ready for read/write as determined by the values in the fd_set.
    *
    * \param    set         Set used in the select statement
    * \return   True if the fd is set. False otherwise.
    */
    bool isReady(fd_set& set) const;

private:

    mutable struct termios oldSettings;     ///< Save the old settings, so they can be returned when the line closes
    std::string            dev;             ///< Serial device to open and talk with
    Baud                   baud;            ///< Baud rate of the connection
    SerialMode             parityMode;      ///< Parity for the connection
    bool                   useCanonical;    ///< Flag indicating if canonical mode should be used
    char                   eol;             ///< End-of-line character to use if operating in canonical mode
    mutable int            fd;              ///< Low-level file descriptor for serial accesses
};

} // namespace utils
} // namespace vulcan

#endif // UTILS_SERIAL_H
