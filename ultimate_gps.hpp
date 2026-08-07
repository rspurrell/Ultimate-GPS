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

#include <cstdint> // The cstdint header provides fixed-width integer types for hardware identifiers.
#include <string> // The string header provides std::string for configurable device paths.

#include "ultimate_gps_types.hpp"

// Forward declarations to avoid polluting consumer namespace with libgpiod headers
struct gpiod_chip;
struct gpiod_line_request;

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

        /**
         * @brief Returns the latest GPS navigation state.
         *
         * @return Constant reference to current GPS data.
         */
        const GpsData& GetData() const noexcept;

        /**
         * @brief Returns the current GPS fix status according to the latest RMC sentence.
         *
         * @return One of the following values:
         *         - Initializing when the receiver is performing a cold start
         *         - NoLock when a previously acquired lock was lost
         *         - Locked when the latest RMC sentence reports a valid fix
         */
        GpsStatus GetStatus() const noexcept;

        /**
         * @brief Returns whether at least one PPS edge has been detected.
         *
         * @return True when PPS has been received.
         */
        bool HasReceivedPps() const noexcept;

        /**
         * @brief Returns the PPS event count.
         *
         * @return Number of PPS events since Open().
         */
        std::uint32_t GetPpsEventCount() const noexcept;

        /**
         * @brief Returns the latest PPS event.
         *
         * @return Constant reference to the latest PPS event.
         */
        const PpsEvent& GetLastPpsEvent() const noexcept;

    private:

        /**
         * @brief Opens and configures the Linux UART serial device for non-blocking raw communication.
         *
         * @return True when the serial port is opened and termios options are set successfully.
         */
        bool OpenSerial();

        /**
         * @brief Requests and configures the GPS PPS GPIO line for rising-edge event detection via libgpiod v2.
         *
         * @return True when the line request succeeds.
         */
        bool OpenPpsGpio();

        /**
         * @brief Reads all available UART data without blocking.
         *
         * @return True when bytes were received.
         */
        bool ReadSerialData();

        /**
         * @brief Scans receiveBuffer_ for complete NMEA sentences, validates checksums, and parses contents.
         *
         * @return True when at least one complete sentence was processed.
         */
        bool ProcessReceiveBuffer();

        /**
         * @brief Reads pending rising-edge events from the PPS GPIO line and updates PPS timing state.
         *
         * @return True when one or more edge events were read and recorded.
         */
        bool ProcessPpsEvents();

        /// Linux file descriptor for the non-blocking UART serial connection. Set to -1 when closed or uninitialized.
        int fdSerial_{-1};

        /// Handle to the libgpiod v2 GPIO chip device.
        gpiod_chip* gpioChip_{nullptr};

        /// Handle to the libgpiod v2 line request for PPS rising-edge event monitoring.
        gpiod_line_request* ppsRequest_{nullptr};

        /// Accumulation buffer for raw incoming UART characters prior to NMEA line parsing.
        std::string receiveBuffer_{};

        /// Configuration settings for UART device paths, baud rate, and GPIO lines.
        GpsConfig config_{};

        /// Cached current state of GPS navigation data, fix statuses, and time values.
        GpsData data_{};

        /// Stores the details and timestamp of the most recent PPS rising-edge event.
        PpsEvent lastPpsEvent_{};

        /// Indicates whether at least one valid hardware PPS pulse event has been recorded.
        bool receivedPps_{false};

        /// Running count of detected hardware PPS pulse events since the interface was opened.
        std::uint32_t ppsEventCount_{0U};
    };
}