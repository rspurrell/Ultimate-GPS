/******************************************************************************
 * @file    ultimate_gps_utils.cpp
 * @brief   Contains utility functions for converting raw sensor measurements.
 ******************************************************************************/

#include "ultimate_gps_utils.hpp"

#include <cmath> // Provides math functions for NMEA coordinate parsing
#include <charconv> // Provides std::from_chars for string_view parsing
#include <cstdint> // Provides fixed-width integer types (uint8_t, uint16_t, etc.).
#include <limits> // Provides access to numeric_limits lowest() and max().
#include <string> // The string header provides std::string.
#include <vector> // Include the vector header for NMEA field storage.

namespace GPSUtils
{
    /// Identifies the beginning of an NMEA sentence for sentence framing.
    inline constexpr char kNmeaStartCharacter = '$';
    /// Separates the NMEA sentence payload from its hexadecimal checksum.
    inline constexpr char kNmeaChecksumDelimiter = '*';
    /// Separates individual fields within an NMEA sentence.
    inline constexpr char kNmeaFieldDelimiter = ',';
    /// Identifies south latitude coordinates in NMEA coordinate fields.
    inline constexpr char kSouthHemisphere = 'S';
    /// Identifies west longitude coordinates in NMEA coordinate fields.
    inline constexpr char kWestHemisphere = 'W';

    /**
     * @brief Defines the number of hexadecimal characters used by an NMEA checksum.
     *
     * NMEA represents its checksum as exactly two hexadecimal characters following
     * the checksum delimiter.
     */
    inline constexpr std::size_t kNmeaChecksumCharacterCount = 2U;

    /**
     * @brief Defines the number of minutes contained in one degree of latitude or longitude.
     *
     * NMEA represents geographic coordinates using degrees and minutes, requiring
     * this value when converting coordinates into decimal degrees.
     */
    inline constexpr double kNmeaMinutesPerDegree = 60.0;

    /**
     * @brief Defines the number used to separate NMEA degree and minute fields.
     *
     * NMEA coordinates use DDMM.MMMM or DDDMM.MMMM notation.
     */
    inline constexpr double kNmeaDegreesToMinutesFactor = 100.0;

    /**
     * @brief Defines the two-digit NMEA year epoch.
     *
     * The GPS reports a two-digit year. We map the value to the 2000 epoch
     * because this library targets the modern GPS receiver use case.
     */
    inline constexpr std::uint16_t kNmeaYearEpoch = 2000U;

    /**
     * @brief Defines the conversion factor from knots to meters per second.
     *
     * NMEA reports GPS ground speed in knots, while the GPS library exposes
     * velocity using SI units.
     */
    inline constexpr double kKnotsToMetersPerSecond = 0.5144444444444444;

    /**
     * @brief Represents an invalid floating-point result when coordinate parsing fails.
     *
     * A quiet NaN allows callers to distinguish an invalid coordinate from a valid
     * coordinate whose value happens to be zero.
     */
    inline constexpr double kInvalidDoubleValue = std::numeric_limits<double>::quiet_NaN();

    /**
     * @brief Size of GPS sentence type identifier (e.g. RMC in $GPRMC)
     */
    inline constexpr std::size_t kSentenceTypeSuffixSize = 3U;

    /**
     * @brief Identifies the RMC NMEA sentence type.
     *
     * The suffix is used because the talker identifier preceding RMC can vary
     * between GPS receivers and NMEA sentence variants.
     */
    inline constexpr std::string_view kRmcSentenceSuffix = "RMC";

    /**
     * @brief Identifies the GGA NMEA sentence type.
     *
     * The suffix is used because the talker identifier preceding GGA can vary
     * between GPS receivers and NMEA sentence variants.
     */
    inline constexpr std::string_view kGgaSentenceSuffix = "GGA";

    /**
     * @brief Identifies the GSA NMEA sentence type.
     *
     * The suffix is used because the talker identifier preceding GSA can vary
     * between GPS receivers and NMEA sentence variants.
     */
    inline constexpr std::string_view kGsaSentenceSuffix = "GSA";

    bool ParseNmeaSentence(UltimateGPS::GpsData& data, std::string_view sentence)
    {
        if (!ValidateNmeaChecksum(sentence))
        {
            return false;
        }

        const std::vector<std::string_view> fields = SplitNmeaFields(sentence);

        if (fields.empty())
        {
            return false;
        }

        std::string_view type = fields.front();
        if (type.size() < kSentenceTypeSuffixSize)
        {
            return false;
        }

        std::string_view suffix = type.substr(type.size() - kSentenceTypeSuffixSize);

        if (suffix == kRmcSentenceSuffix)
        {
            return ParseRmc(data, fields);
        }

        if (suffix == kGgaSentenceSuffix)
        {
            return ParseGga(data, fields);
        }

        if (suffix == kGsaSentenceSuffix)
        {
            return ParseGsa(data, fields);
        }

        return false;
    }

    bool ValidateNmeaChecksum(std::string_view sentence)
    {
        if (sentence.empty() || sentence.front() != kNmeaStartCharacter)
        {
            return false;
        }

        const std::size_t chkSumDelimiterIdx = sentence.find(kNmeaChecksumDelimiter);
        if (chkSumDelimiterIdx == std::string::npos)
        {
            return false;
        }

        // Ensure that checksum is two hex characters
        const std::size_t checksumStart = chkSumDelimiterIdx + 1U;
        if (sentence.size() - checksumStart < kNmeaChecksumCharacterCount)
        {
            return false;
        }

        std::uint8_t calculatedChecksum = 0U;
        for (std::size_t idx = 1U; idx < chkSumDelimiterIdx; idx++)
        {
            calculatedChecksum ^= static_cast<std::uint8_t>(sentence[idx]);
        }

        const int highNibble = HexToValue(sentence[checksumStart]);
        const int lowNibble = HexToValue(sentence[checksumStart + 1U]);
        if (highNibble < 0 || lowNibble < 0)
        {
            return false;
        }
        const std::uint8_t receivedChecksum = static_cast<std::uint8_t>((highNibble << 4) | lowNibble);

        return calculatedChecksum == receivedChecksum;
    }

    std::vector<std::string_view> SplitNmeaFields(std::string_view sentence)
    {
        static constexpr std::size_t payloadStartOffset = 1U;

        // validation has already been performed. Find will always succeed.
        const std::size_t chkSumIdx = sentence.find(kNmeaChecksumDelimiter);
        const std::size_t payloadEnd = chkSumIdx;
        std::size_t fieldStart = payloadStartOffset;

        std::vector<std::string_view> fields;
        for (std::size_t index = payloadStartOffset; index <= payloadEnd; ++index)
        {
            if (index == payloadEnd || sentence[index] == kNmeaFieldDelimiter)
            {
                fields.emplace_back(sentence.substr(fieldStart, index - fieldStart));
                fieldStart = index + 1U;
            }
        }

        return fields;
    }

    bool ParseRmc(UltimateGPS::GpsData& data, const std::vector<std::string_view>& fields)
    {
        static constexpr size_t idxTime = 1U, idxStatus = 2U,
            idxLatitude = 3U, idxLatitudeHemisphere = 4U,
            idxLongitude = 5U, idxLongitudeHemisphere = 6U,
            idxSpeed = 7U, idxCourse = 8U, idxDate = 9U,
            idxMagVariation = 10U, idxMagDirection = 11U;

        static constexpr std::size_t kMinimumRmcFields = 10U;
        if (fields.size() < kMinimumRmcFields)
        {
            return false;
        }

        ParseUtcTime(data, fields[idxTime]);

        data.nmeaFixValid = fields[idxStatus] == "A";
        if (!data.nmeaFixValid)
        {
            return true;
        }

        if (!fields[idxLatitude].empty() && !fields[idxLatitudeHemisphere].empty())
        {
            data.latitude = ConvertNmeaCoordinate(fields[idxLatitude], fields[idxLatitudeHemisphere].front());
        }

        if (!fields[idxLongitude].empty() && !fields[idxLongitudeHemisphere].empty())
        {
            data.longitude = ConvertNmeaCoordinate(fields[idxLongitude], fields[idxLongitudeHemisphere].front());
        }

        if (!fields[idxSpeed].empty())
        {
            double speedKnots = 0.0;
            if (TryParseNumber(fields[idxSpeed], speedKnots))
            {
                data.speedMetersPerSecond = speedKnots * kKnotsToMetersPerSecond;
            }
        }

        if (!fields[idxCourse].empty())
        {
            double course = 0.0;
            if (TryParseNumber(fields[idxCourse], course))
            {
                data.courseDegrees = course;
            }
        }

        ParseUtcDate(data, fields[idxDate]);

        if (fields.size() > idxMagDirection)
        {
            double magneticVariation = kInvalidDoubleValue;
            if (TryParseNumber(fields[idxMagVariation], magneticVariation))
            {
                const char direction = fields[idxMagDirection][0];
                data.magneticVariation = magneticVariation;
                if (direction == kWestHemisphere)
                {
                    data.magneticVariation = -data.magneticVariation;
                }
            }
        }

        return true;
    }

    bool ParseGga(UltimateGPS::GpsData& data, const std::vector<std::string_view>& fields)
    {
        static constexpr size_t idxTime = 1U,
            idxLatitude = 2U, idxLatitudeHemisphere = 3U,
            idxLongitude = 4U, idxLongitudeHemisphere = 5U,
            idxFixQuality = 6U, idxSatellites = 7U, idxHDOP = 8U,
            idxAltitude = 9U, idxGeoidSep = 11U;

        static constexpr std::size_t kMinimumGgaFields = 10U;
        if (fields.size() < kMinimumGgaFields)
        {
            return false;
        }

        ParseUtcTime(data, fields[idxTime]);

        if (!fields[idxLatitude].empty() && !fields[idxLatitudeHemisphere].empty())
        {
            data.latitude = ConvertNmeaCoordinate(fields[idxLatitude], fields[idxLatitudeHemisphere].front());
        }

        if (!fields[idxLongitude].empty() && !fields[idxLongitudeHemisphere].empty())
        {
            data.longitude = ConvertNmeaCoordinate(fields[idxLongitude], fields[idxLongitudeHemisphere].front());
        }

        std::uint32_t fixQuality = 0U;
        if (!fields[idxFixQuality].empty())
        {
            TryParseNumber(fields[idxFixQuality], fixQuality);
        }
        data.nmeaPositionValid = fixQuality > 0U;

        if (!fields[idxSatellites].empty())
        {
            std::uint32_t satellites = 0U;
            if (TryParseNumber(fields[idxSatellites], satellites))
            {
                // Clamp to 255
                static constexpr std::uint32_t maximumSatelliteValue = std::numeric_limits<std::uint8_t>::max();
                satellites = std::min(satellites, maximumSatelliteValue);
                data.satellites = static_cast<std::uint8_t>(satellites);
            }
        }

        if (!fields[idxHDOP].empty())
        {
            double hdop = kInvalidDoubleValue;
            if (TryParseNumber(fields[idxHDOP], hdop))
            {
                data.horizontalDOP = hdop;
            }
        }

        if (!fields[idxAltitude].empty())
        {
            double altitude = 0.0;
            if (TryParseNumber(fields[idxAltitude], altitude))
            {
                data.altitudeMeters = altitude;
            }
        }

        if (fields.size() > idxGeoidSep && !fields[idxGeoidSep].empty())
        {
            double geoidSeparation = kInvalidDoubleValue;
            if (TryParseNumber(fields[idxGeoidSep], geoidSeparation))
            {
                data.geoidSeparation = geoidSeparation;
            }
        }

        return true;
    }

    bool ParseGsa(UltimateGPS::GpsData& data, const std::vector<std::string_view>& fields)
    {
        static constexpr size_t kFixModeFieldIndex = 2U;

        static constexpr size_t kMinimumGsaFields = 3U;
        if (fields.size() < kMinimumGsaFields)
        {
            return false;
        }

        std::uint32_t fixMode = 0U;
        if (!TryParseNumber(fields[kFixModeFieldIndex], fixMode))
        {
            return false;
        }

        switch (fixMode)
        {
        case 2U:
            data.fixType = UltimateGPS::FixType::TwoDimensional;
            break;
        case 3U:
            data.fixType = UltimateGPS::FixType::ThreeDimensional;
            break;
        default:
            data.fixType = UltimateGPS::FixType::None;
            break;
        }

        return true;
    }

    double ConvertNmeaCoordinate(std::string_view coordinate, char hemisphere)
    {
        double rawCoordinate = 0.0;

        if (!TryParseNumber(coordinate, rawCoordinate))
        {
            return 0.0;
        }

        const double degrees = std::floor(rawCoordinate / kNmeaDegreesToMinutesFactor);
        const double minutes = rawCoordinate - (degrees * kNmeaDegreesToMinutesFactor);
        double decimalDegrees = degrees + (minutes / kNmeaMinutesPerDegree);

        if (hemisphere == kSouthHemisphere || hemisphere == kWestHemisphere)
        {
            decimalDegrees = -decimalDegrees;
        }

        return decimalDegrees;
    }

    void ParseUtcDate(UltimateGPS::GpsData& data, std::string_view value)
    {
        if (value.size() < 6U)
        {
            return;
        }

        std::uint32_t day = 0U;
        std::uint32_t month = 0U;
        std::uint32_t year = 0U;

        if (!TryParseNumber(value.substr(0U, 2U), day))
        {
            return;
        }

        if (!TryParseNumber(value.substr(2U, 2U), month))
        {
            return;
        }

        if (!TryParseNumber(value.substr(4U, 2U), year))
        {
            return;
        }

        data.utcDay = static_cast<std::uint8_t>(day);
        data.utcMonth = static_cast<std::uint8_t>(month);
        data.utcYear = static_cast<std::uint16_t>(kNmeaYearEpoch + year);
    }

    void ParseUtcTime(UltimateGPS::GpsData& data, std::string_view value)
    {
        if (value.size() < 6U)
        {
            return;
        }

        std::uint32_t hour = 0U;
        std::uint32_t minute = 0U;
        std::uint32_t second = 0U;

        if (!TryParseNumber(value.substr(0U, 2U), hour))
        {
            return;
        }

        if (!TryParseNumber(value.substr(2U, 2U), minute))
        {
            return;
        }

        if (!TryParseNumber(value.substr(4U, 2U), second))
        {
            return;
        }

        data.utcHour = static_cast<std::uint8_t>(hour);
        data.utcMinute = static_cast<std::uint8_t>(minute);
        data.utcSecond = static_cast<std::uint8_t>(second);
    }

    template<typename T>
    bool TryParseNumber(std::string_view value, T& result)
    {
        // Fails compilation if T is not an integer or floating-point type
        static_assert(std::is_arithmetic_v<T>, "TryParseNumber only supports numeric types.");

        if (value.empty())
        {
            return false;
        }

        auto [ptr, ec] = std::from_chars(value.data(), value.data() + value.size(), result);

        // Success means no error (ec) and the entire string was consumed (ptr == end)
        return ec == std::errc{} && ptr == value.data() + value.size();
    }

    int HexToValue(char value)
    {
        static constexpr char kDigitZero = '0';
        static constexpr char kDigitNine = '9';
        static constexpr char kUppercaseA = 'A';
        static constexpr char kUppercaseF = 'F';
        static constexpr char kLowercaseA = 'a';
        static constexpr char kLowercaseF = 'f';
        static constexpr int kUppercaseOffset = 10;
        static constexpr int kLowercaseOffset = 10;

        if (value >= kDigitZero && value <= kDigitNine)
        {
            return value - kDigitZero;
        }

        if (value >= kUppercaseA && value <= kUppercaseF)
        {
            return value - kUppercaseA + kUppercaseOffset;
        }

        if (value >= kLowercaseA && value <= kLowercaseF)
        {
            return value - kLowercaseA + kLowercaseOffset;
        }

        return -1;
    }
}