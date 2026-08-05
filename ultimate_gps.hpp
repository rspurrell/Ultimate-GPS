/**
 * @file ultimate_gps.hpp
 * @brief Standalone interface for the Adafruit Ultimate GPS v3.
 *
 * This library provides a reusable Linux C++ interface for the
 * Adafruit Ultimate GPS v3 breakout board.
 *
 * The library communicates with the GPS through:
 *
 * - UART for NMEA navigation sentences.
 * - PPS GPIO for precision one-pulse-per-second timing.
 */

#pragma once

#include <string> // The string header provides std::string for configurable device paths.

#include "ultimate_gps_types.hpp"

/**
 * @brief Contains the standalone Ultimate GPS library.
 */
namespace UltimateGPS
{
    /**
     * @brief Provides a standalone interface to the Adafruit Ultimate GPS v3.
     *
     * UART communication is non-blocking.
     * PPS is monitored through kernel-generated GPIO edge events.
     */
    class GPS
    {
    public:

        /**
         * @brief Constructs a GPS interface using default configuration.
         */
        GPS();

        /**
         * @brief Constructs a GPS interface using custom configuration.
         *
         * @param config GPS configuration.
         */
        explicit GPS(const GpsConfig& config);

        /**
         * @brief Releases all GPS resources.
         */
        ~GPS();

        /**
         * @brief Prevents copying of the resource-owning GPS interface.
         */
        GPS(const GPS&) = delete;

        /**
         * @brief Prevents copy assignment of the resource-owning GPS interface.
         */
        GPS& operator=(const GPS&) = delete;

        /**
         * @brief Opens the UART and GPIO interfaces.
         *
         * @return True when initialization succeeds.
         */
        bool Open();

        /**
         * @brief Closes the UART and GPIO interfaces.
         */
        void Close();

        /**
         * @brief Determines whether the GPS interface is open.
         *
         * @return True when the GPS interface is active.
         */
        bool IsOpen() const noexcept;

        /**
         * @brief Processes all currently available GPS input.
         *
         * This method never intentionally waits for future UART or PPS data.
         *
         * @return True when at least one input event was processed.
         */
        bool Update();
    private:

        /**
         * @brief Opens and configures the Linux UART serial device for non-blocking raw communication.
         *
         * @return True when the serial port is opened and termios options are set successfully.
         */
        bool OpenSerial();

        /**
         * @brief Reads all available UART data without blocking.
         *
         * @return True when bytes were received.
         */
        bool ReadSerialData();

        /// Linux file descriptor for the non-blocking UART serial connection. Set to -1 when closed or uninitialized.
        int fdSerial_{-1};

        /// Accumulation buffer for raw incoming UART characters prior to NMEA line parsing.
        std::string receiveBuffer_{};

        /// Configuration settings for UART device paths, baud rate, and GPIO lines.
        GpsConfig config_{};
    };
}