/**
 * @file ultimate_gps.cpp
 * @brief Implementation of the standalone Adafruit Ultimate GPS v3 library.
 */

#include "ultimate_gps.hpp"
#include "ultimate_gps_utils.hpp"

// Include errno values used to distinguish non-blocking reads from errors.
#include <cerrno>

// Include the C standard library numeric conversion functions.
#include <cstdlib>

// Include the C standard library string utilities.
#include <cstring>

// Include the Linux file-control API used to open the UART.
#include <fcntl.h>

// Include the Linux terminal API used to configure the UART.
#include <termios.h>

// Include the Linux POSIX API used to read and close file descriptors.
#include <unistd.h>

// Include the libgpiod v2 C API used to access GPIO character devices.
#include <gpiod.h>

// Include the algorithm header for std::floor.
#include <cmath>

// Include the limits header for numeric range validation.
#include <limits>

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

    /** @brief Named constant representing the maximum number of GPIO events retrieved from a single PPS event read operation. */
    inline constexpr std::size_t kPpsEventBufferSize = 16U;

    /**
     * @brief Named constant representing the default GPIO consumer label.
     *
     * The label appears in Linux GPIO diagnostics and helps identify which
     * application currently owns the GPS GPIO lines.
     */
    inline constexpr const char* kGpioConsumerName = "ultimate-gps";

    /** @brief Defines the optional NMEA carriage-return character. */
    inline constexpr char kNmeaCarriageReturnCharacter = '\r';

    /** @brief Defines the NMEA line-feed terminator. */
    inline constexpr char kNmeaLineFeedCharacter = '\n';

    inline constexpr std::int64_t kPpsWaitTimeoutMilliseconds = 0;

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

    /**
     * @brief Converts a libgpiod edge-event timestamp to a chrono timestamp.
     *
     * @param eventTimestamp Timestamp supplied by libgpiod.
     *
     * @return Monotonic timestamp in nanoseconds.
     */
    inline std::chrono::nanoseconds ToChronoTimestamp(std::uint64_t eventTimestamp)
    {
        return std::chrono::nanoseconds(
            static_cast<std::chrono::nanoseconds::rep>(eventTimestamp)
        );
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

        if (!OpenPpsGpio())
        {
            Close();
            return false;
        }

        receiveBuffer_.clear();
        data_ = GpsData{};
        lastPpsEvent_ = PpsEvent{};
        receivedPps_ = false;
        ppsEventCount_ = 0U;

        return true;
    }

    void GPS::Close()
    {
        if (ppsRequest_ != nullptr)
        {
            gpiod_line_request_release(ppsRequest_);
            ppsRequest_ = nullptr;
        }

        if (gpioChip_ != nullptr)
        {
            gpiod_chip_close(gpioChip_);
            gpioChip_ = nullptr;
        }

        if (fdSerial_ > kInvalidFileDescriptor)
        {
            close(fdSerial_);
            fdSerial_ = kInvalidFileDescriptor;
        }

        receiveBuffer_.clear();
    }

    bool GPS::IsOpen() const noexcept
    {
        return fdSerial_ >= 0 &&
               gpioChip_ != nullptr &&
               ppsRequest_ != nullptr;
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

        if (ProcessReceiveBuffer())
        {
            processed = true;
        }

        if (ProcessPpsEvents())
        {
            processed = true;
        }

        return processed;
    }

    const GpsData& GPS::GetData() const noexcept
    {
        return data_;
    }

    GpsStatus GPS::GetStatus() const noexcept
    {
        return data_.status;
    }

    bool GPS::HasReceivedPps() const noexcept
    {
        return receivedPps_;
    }

    std::uint32_t GPS::GetPpsEventCount() const noexcept
    {
        return ppsEventCount_;
    }

    const PpsEvent& GPS::GetLastPpsEvent() const noexcept
    {
        return lastPpsEvent_;
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

    bool GPS::OpenPpsGpio()
    {
        // Prevent multi-call resource leaks and EBUSY errors
        if (ppsRequest_ != nullptr)
        {
            return true; // Already opened successfully
        }

        // Open the GPIO chip if not already opened. The chip represents the GPIO controller device.
        if (gpioChip_ == nullptr)
        {
            gpioChip_ = gpiod_chip_open(config_.gpioChip.c_str());
            if (gpioChip_ == nullptr)
            {
                return false;
            }
        }

        // Create a new line settings object to configure the GPIO line properties.
        gpiod_line_settings* settings = gpiod_line_settings_new();
        if (settings == nullptr)
        {
            return false;
        }

        // Configure the GPIO line as an input to receive pulse-per-second (PPS) signals.
        if (gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT) != 0)
        {
            gpiod_line_settings_free(settings);
            return false;
        }

        // Set edge detection to rising edge to trigger on the PPS pulse transition from low to high.
        if (gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING) != 0)
        {
            gpiod_line_settings_free(settings);
            return false;
        }

        // Set event clock to monotonic as our pulse-per-second event measurement should be relative to init runtime.
        if (gpiod_line_settings_set_event_clock(settings, GPIOD_LINE_CLOCK_MONOTONIC) != 0)
        {
            gpiod_line_settings_free(settings);
            return false;
        }

        // Create a line configuration object that will hold the settings for one or more GPIO lines.
        gpiod_line_config* lineConfig = gpiod_line_config_new();
        if (lineConfig == nullptr)
        {
            gpiod_line_settings_free(settings);
            return false;
        }

        // Add the configured settings to the line configuration for the specific GPIO line offset.
        const unsigned int offset = config_.ppsGpio;
        if (gpiod_line_config_add_line_settings(lineConfig, &offset, 1U, settings) != 0)
        {
            gpiod_line_settings_free(settings);
            gpiod_line_config_free(lineConfig);
            return false;
        }

        // Free the line settings object as it has been copied into the line configuration.
        gpiod_line_settings_free(settings);

        // Create a request configuration object to configure the request parameters.
        gpiod_request_config* requestConfig = gpiod_request_config_new();
        if (requestConfig == nullptr)
        {
            gpiod_line_config_free(lineConfig);
            return false;
        }

        // Match kernel buffer to user-space buffer
        gpiod_request_config_set_event_buffer_size(requestConfig, kPpsEventBufferSize);

        // Set the consumer name for tracking which application/service owns this GPIO line request.
        gpiod_request_config_set_consumer(requestConfig, kGpioConsumerName);

        // Request the GPIO line from the chip with the configured settings. Store the request handle for later use.
        ppsRequest_ = gpiod_chip_request_lines(gpioChip_, requestConfig, lineConfig);

        // Free the request and line configuration objects as they are no longer needed.
        gpiod_request_config_free(requestConfig);
        gpiod_line_config_free(lineConfig);

        // Return success if the request handle was obtained; otherwise return false indicating failure.
        return ppsRequest_ != nullptr;
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

    bool GPS::ProcessReceiveBuffer()
    {
        bool processed = false;
        while (true)
        {
            const std::size_t newlinePosition = receiveBuffer_.find(kNmeaLineFeedCharacter);
            if (newlinePosition == std::string::npos)
            {
                break;
            }

            std::string sentence = receiveBuffer_.substr(0U, newlinePosition);
            receiveBuffer_.erase(0U, newlinePosition + 1U);
            if (!sentence.empty() && sentence.back() == kNmeaCarriageReturnCharacter)
            {
                sentence.pop_back();
            }

            if (GPSUtils::ParseNmeaSentence(data_, sentence))
            {
                processed = true;
            }
        }

        return processed;
    }

    bool GPS::ProcessPpsEvents()
    {
        if (ppsRequest_ == nullptr)
        {
            return false;
        }

        const int waitResult = gpiod_line_request_wait_edge_events(ppsRequest_, kPpsWaitTimeoutMilliseconds);
        if (waitResult <= 0)
        {
            return false;
        }

        gpiod_edge_event_buffer* eventBuffer = gpiod_edge_event_buffer_new(kPpsEventBufferSize);
        if (eventBuffer == nullptr)
        {
            return false;
        }

        const int numEvents = gpiod_line_request_read_edge_events(ppsRequest_, eventBuffer, kPpsEventBufferSize);
        if (numEvents < 0)
        {
            gpiod_edge_event_buffer_free(eventBuffer);
            return false;
        }

        bool processed = false;
        for (size_t i = 0; i < static_cast<size_t>(numEvents); ++i)
        {
            gpiod_edge_event* event = gpiod_edge_event_buffer_get_event(eventBuffer, i);
            if (event == nullptr)
            {
                continue;
            }

            if (gpiod_edge_event_get_event_type(event) != GPIOD_EDGE_EVENT_RISING_EDGE)
            {
                continue;
            }

            lastPpsEvent_.valid = true;
            lastPpsEvent_.timestamp = ToChronoTimestamp(gpiod_edge_event_get_timestamp_ns(event));
            lastPpsEvent_.sequence = ++ppsEventCount_;
            receivedPps_ = true;
            processed = true;
        }

        gpiod_edge_event_buffer_free(eventBuffer);

        return processed;
    }
}