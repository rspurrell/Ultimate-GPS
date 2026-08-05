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

    /**
     * @brief Represents the current GPS navigation state.
     *
     * The structure contains both decoded NMEA values and independently
     * observed hardware status values.
     */
    struct GpsData
    {

        /** @brief Indicates whether RMC reports a valid navigation fix. */
        bool nmeaFixValid{false};

        /**
         * @brief Stores the latitude in decimal degrees.
         *
         * Positive values represent north.
         * Negative values represent south.
         */
        double latitude{0.0};

        /**
         * @brief Stores the longitude in decimal degrees.
         *
         * Positive values represent east.
         * Negative values represent west.
         */
        double longitude{0.0};

        /** @brief Ground speed in meters per second. */
        double speedMetersPerSecond{0.0};

        /** @brief Course over ground in degrees relative to true north. */
        double courseDegrees{0.0};

        /**
         * @brief Magnetic variation in degrees.
         *
         * @details Positive values indicate east variation and negative values indicate west
         * variation. This value represents the angular difference between true north
         * and magnetic north at the current location.
         *
         * @note This field is optional and may not be provided by all GPS receivers.
         */
        double magneticVariation{0.0};

        /** @brief UTC year. */
        std::uint16_t utcYear{0U};

        /** @brief UTC month. */
        std::uint8_t utcMonth{0U};

        /** @brief UTC day. */
        std::uint8_t utcDay{0U};

        /** @brief UTC hour. */
        std::uint8_t utcHour{0U};

        /** @brief UTC minute. */
        std::uint8_t utcMinute{0U};

        /** @brief UTC second. */
        std::uint8_t utcSecond{0U};
    };
}