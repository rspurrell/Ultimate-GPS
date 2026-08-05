/******************************************************************************
 * @file    ultimate_gps_utils.hpp
 * @brief   Contains utility functions for converting raw sensor measurements.
 ******************************************************************************/

#pragma once

#include <string> // The string header provides std::string.
#include <vector> // Include the vector header for NMEA field storage.

#include "ultimate_gps_types.hpp"

namespace GPSUtils
{
    /**
     * @brief Parses a complete NMEA sentence.
     *
     * @param data The GPS data to update.
     * @param sentence Complete NMEA sentence.
     *
     * @return True when the sentence is valid and recognized.
     */
    bool ParseNmeaSentence(UltimateGPS::GpsData& data, std::string_view sentence);

    /**
     * @brief Splits a validated NMEA sentence into comma-delimited fields.
     *
     * The checksum delimiter is required because a complete NMEA sentence must
     * contain a checksum. A sentence without the checksum delimiter is malformed
     * and therefore cannot be safely parsed.
     *
     * Empty fields are preserved because their positions have defined meanings in
     * the NMEA protocol.
     *
     * @param sentence Complete NMEA sentence including its checksum.
     *
     * @return Vector containing each comma-separated field, or an empty vector
     *         when the sentence does not contain the required checksum delimiter.
     */
    std::vector<std::string_view> SplitNmeaFields(std::string_view sentence);

    /**
     * @brief Parses an RMC sentence.
     *
     * @param data The GPS data to update RMC fields.
     * @param fields Parsed NMEA fields.
     *
     * @return True when parsing succeeds.
     */
    bool ParseRmc(UltimateGPS::GpsData& data, const std::vector<std::string_view>& fields);

    /**
     * @brief Converts an NMEA coordinate into decimal degrees.
     *
     * @param coordinate NMEA DDMM.MMMM or DDDMM.MMMM coordinate.
     * @param hemisphere Hemisphere character.
     *
     * @return Decimal-degree coordinate.
     */
    double ConvertNmeaCoordinate(std::string_view coordinate, char hemisphere);

    /**
     * @brief Parses UTC date from an NMEA ddmmyy field.
     *
     * @param data The GPS data to update UTC date fields.
     * @param value NMEA UTC date.
     */
    void ParseUtcDate(UltimateGPS::GpsData& data, std::string_view value);

    /**
     * @brief Parses UTC time from an NMEA hhmmss.sss field.
     *
     * @param data The GPS data to update UTC time fields.
     * @param value NMEA UTC time.
     */
    void ParseUtcTime(UltimateGPS::GpsData& data, std::string_view value);

    /**
     * @brief Parses an unsigned integer without throwing exceptions.
     *
     * @param value Input string.
     * @param result Destination value.
     *
     * @return True when parsing succeeds.
     */
    template<typename T>
    bool TryParseNumber(std::string_view value, T& result);

}