/**
 * @file gps_types.hpp
 * @brief Public data types used by the standalone GPS library.
 *
 * These types contain no Linux-specific implementation details so that the
 * public GPS data model remains independent of the underlying transport.
 */

#pragma once

#include <cstdint>
#include <string>

namespace UltimateGPS
{
    /**
     * @brief Default Linux UART device.
     */
    inline constexpr const char* kDefaultSerialDevice = "/dev/serial0";

    /**
     * @brief Default GPS UART baud rate.
     */
    inline constexpr std::uint32_t kDefaultBaudRate = 9600U;

    /**
     * @brief Configures the GPS UART and GPIO interfaces.
     */
    struct GpsConfig
    {
        /**
         * @brief Linux serial device used by the GPS UART.
         */
        std::string serialDevice{kDefaultSerialDevice};

        /**
         * @brief GPS UART baud rate.
         */
        std::uint32_t baudRate{kDefaultBaudRate};

    };
}