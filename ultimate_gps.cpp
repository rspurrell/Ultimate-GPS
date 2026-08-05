/**
 * @file ultimate_gps.cpp
 * @brief Implementation of the standalone Adafruit Ultimate GPS v3 library.
 */

#include "ultimate_gps.hpp"

// Include the Linux file-control API used to open the UART.
#include <fcntl.h>

// Include the Linux terminal API used to configure the UART.
#include <termios.h>

// Include the Linux POSIX API used to read and close file descriptors.
#include <unistd.h>

/**
 * @brief Contains the implementation of the standalone GPS library.
 */
namespace UltimateGPS
{
    /** @brief Named constant representing an invalid Linux file descriptor. */
    inline constexpr int kInvalidFileDescriptor = -1;

    /** @brief Named constant representing a successful POSIX return value. */
    inline constexpr int kPosixSuccess = 0;

    /** @brief Named constant representing the number of UART bytes read per non-blocking read operation. */
    inline constexpr std::size_t kSerialReadBufferSize = 512U;

    /**
     * @brief Converts the configured baud rate to the Linux termios constant.
     *
     * @param baudRate Requested baud rate.
     *
     * @return Matching termios constant, or zero when unsupported.
     */
    inline speed_t ToTermiosBaudRate(std::uint32_t baudRate)
    {
        switch (baudRate)
        {
        case 4800U:
            return B4800;

        // Default. Ideal for standard 1Hz update rates or basic RMC sentences.
        case 9600U:
            return B9600;

        case 19200U:
            return B19200;

        // Recommended minimum speed if GPS update rate increased to 5Hz or 10Hz.
        case 38400U:
            return B38400;

        // Commonly used configuration for higher refresh rates.
        case 57600U:
            return B57600;

        // Maximum recommended speed for stable, heavy data streaming at full 10Hz update rates
        case 115200U:
            return B115200;

        default:
            return 0;
        }
    }


    GPS::GPS() : config_(GpsConfig{}) {}

    GPS::GPS(const GpsConfig& config) : config_(config) {}

    GPS::~GPS()
    {
        Close();
    }

    bool GPS::Open()
    {
        if (IsOpen())
        {
            return true;
        }

        if (!OpenSerial())
        {
            Close();
            return false;
        }

        receiveBuffer_.clear();
        return true;
    }

    void GPS::Close()
    {
        if (fdSerial_ > kInvalidFileDescriptor)
        {
            close(fdSerial_);
            fdSerial_ = kInvalidFileDescriptor;
        }

        receiveBuffer_.clear();
    }

    bool GPS::IsOpen() const noexcept
    {
        return fdSerial_ >= 0;
    }

    bool GPS::Update()
    {
        if (!IsOpen())
        {
            return false;
        }

        bool processed = false;

        if (ReadSerialData())
        {
            processed = true;
        }

        return processed;
    }

    bool GPS::OpenSerial()
    {
        const speed_t baudRate = ToTermiosBaudRate(config_.baudRate);
        if (baudRate == 0)
        {
            return false;
        }

        fdSerial_ = open(config_.serialDevice.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fdSerial_ <= kInvalidFileDescriptor)
        {
            return false;
        }

        termios settings{};
        if (tcgetattr(fdSerial_, &settings) != kPosixSuccess)
        {
            return false;
        }

        // Put the terminal in raw mode so data is received and transmitted exactly
        // as-is, with no processing, echoing, or line buffering.
        /*
            Equivalent to:
            // Disable input processing features that can alter raw GPS data.
            c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
            // Disable output processing so bytes are sent unchanged.
            c_oflag &= ~OPOST;
            // Disable canonical mode, echo, and signal generation so input is raw.
            c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            // Disable parity generation and checking and clear any existing character size bits before setting data bits.
            c_cflag &= ~(CSIZE | PARENB);
            // Use 8 data bits per byte.
            c_cflag |= CS8;
        */
        cfmakeraw(&settings);

        // Set the input and output baud rate to the configured GPS speed.
        cfsetispeed(&settings, baudRate);
        cfsetospeed(&settings, baudRate);

        // Enable receiver and ignore modem control lines.
        settings.c_cflag |= CLOCAL | CREAD;
        // Use one stop bit.
        settings.c_cflag &= ~CSTOPB;
        // Disable hardware flow control (RTS/CTS).
        settings.c_cflag &= ~CRTSCTS;

        // Configure read operations to be non-blocking with no minimum byte count.
        settings.c_cc[VMIN] = 0U;
        settings.c_cc[VTIME] = 0U;

        if (tcsetattr(fdSerial_, TCSANOW, &settings) != kPosixSuccess)
        {
            return false;
        }

        tcflush(fdSerial_, TCIFLUSH);

        return true;
    }

    bool GPS::ReadSerialData()
    {
        bool dataRead = false;
        char buffer[kSerialReadBufferSize]{};
        while (true)
        {
            const ssize_t bytesRead = read(fdSerial_, buffer, sizeof(buffer));

            if (bytesRead > 0)
            {
                receiveBuffer_.append(buffer, static_cast<std::size_t>(bytesRead));
                dataRead = true;
                continue;
            }

            if (bytesRead == 0)
            {
                break;
            }

            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                break;
            }

            if (errno == EINTR)
            {
                continue;
            }

            break;
        }

        return dataRead;
    }
}