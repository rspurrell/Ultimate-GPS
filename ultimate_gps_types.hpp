/**
 * @file gps_types.hpp
 * @brief Public data types used by the standalone GPS library.
 *
 * These types contain no Linux-specific implementation details so that the
 * public GPS data model remains independent of the underlying transport.
 */

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

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
     * @brief Default Linux GPIO controller.
     */
    inline constexpr const char* kDefaultGpioChip = "/dev/gpiochip0";

    /**
     * @brief Default BCM GPIO used for GPS PPS.
     */
    inline constexpr std::uint32_t kDefaultPpsGpio = 24U;

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

        /**
         * @brief Linux GPIO chip device.
         */
        std::string gpioChip{kDefaultGpioChip};

        /**
         * @brief BCM GPIO connected to GPS PPS.
         */
        std::uint32_t ppsGpio{kDefaultPpsGpio};
    };

    /**
     * @brief Represents the quality of the current GPS navigation solution.
     *
     * The values correspond to the standard NMEA GSA fix-mode values.
     */
    enum class FixType : std::uint8_t
    {
        /** @brief No valid navigation fix is available. */
        None = 1U,

        /** @brief A two-dimensional navigation fix is available. */
        TwoDimensional = 2U,

        /** @brief A three-dimensional navigation fix is available. */
        ThreeDimensional = 3U
    };

    /**
     * @brief Represents a captured PPS event.
     *
     * The timestamp originates from the Linux GPIO character-device event
     * timestamp rather than from userspace clock sampling, minimizing scheduling
     * latency in the reported timestamp.
     */
    struct PpsEvent final
    {
        /** @brief Indicates whether a PPS event was successfully captured. */
        bool valid{false};

        /** @brief Monotonic timestamp at which the kernel observed the PPS edge. */
        std::chrono::nanoseconds timestamp{};

        /** @brief Sequential PPS event number since the GPS was opened. */
        std::uint32_t sequence{0U};
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

        /** @brief Indicates whether GGA reports a valid position. */
        bool nmeaPositionValid{false};

        /** @brief Current NMEA fix classification. */
        FixType fixType{FixType::None};

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

        /** @brief Altitude above mean sea level in meters. */
        double altitudeMeters{0.0};

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

        /**
         * @brief Horizontal dilution of precision.
         *
         * @details A metric that measures how the geometric layout of satellites
         * affects the accuracy of horizontal coordinates (latitude and longitude).
         * Lower values mean better satellite spread and higher accuracy. Clustered
         * satellites yield higher values and more location error.
         */
        double horizontalDOP{0.0};

        /**
         * @brief Geoid Separation represents the difference between the WGS84 ellipsoid
         * height and mean sea level height.
         */
        double geoidSeparation{0.0};

        /** @brief Number of satellites currently reported by the receiver. */
        std::uint16_t satellites{0U};

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

        std::string get32PointHeading() const
        {
            auto degrees = courseDegrees;
            // Normalize input to the range [0.0, 360.0)
            degrees = std::fmod(degrees, 360.0);
            if (degrees < 0.0)
            {
                degrees += 360.0;
            }

            // Define the 32 compass points starting from North (0 degrees)
            static const std::vector<std::string> compassPoints =
            {
                "N", "NbE", "NNE", "NEbN", "NE",  "NEbE", "ENE", "EbN",
                "E", "EbS", "ESE", "SEbE", "SE",  "SEbS", "SSE", "SbE",
                "S", "SbW", "SSW", "SWbS", "SW",  "SWbW", "WSW", "WbS",
                "W", "WbN", "WNW", "NWbW", "NW",  "NWbN", "NNW", "NbW"
            };

            // Shift by half a 32 point cardinal width (11.25 / 2 = 5.625) to center the ranges.
            // Dividing by 11.25 converts the offset degree into an array index integer.
            int index = static_cast<int>((degrees + 5.625) / 11.25);

            // Wrap around index 32 back to 0 (handles the upper half of North)
            return compassPoints[index % 32];
        }
    };
}