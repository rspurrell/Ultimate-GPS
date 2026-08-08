# Ultimate GPS Library

A standalone modern C++ library for interfacing with the **MTK3339 GPS receiver** using the Linux serial interface and GPIO character-device API.

## Features
The library provides access to:

* GPS position and altitude
* Speed, course, and 32-point compass heading
* UTC date and time
* GPS navigation status
* Horizontal dilution of precision (HDOP)
* Geoid separation
* Magnetic variation
* PPS rising-edge event detection
* PPS event timestamps and sequence count

## Requirements

* C++17 or later
* GPS receiver providing NMEA data over a serial interface
* Linux system with serial and GPIO character-device support
* Optional PPS output connected to a GPIO input
* `libgpiod` for PPS GPIO event handling

## Hardware

### Adafruit Ultimate GPS Breakout v3

The library is designed around the Adafruit Ultimate GPS Breakout v3, which uses the MTK3339 GPS receiver.

The GPS communicates with the hardware over a UART serial connection.

The library also supports the module's **PPS** output for precise timing.

### Connections

| GPS Pin | Hardware                   | Purpose                          |
| ------- | -------------------------- | -------------------------------- |
| VIN     | 5V                         | GPS power                        |
| GND     | GND                        | Ground                           |
| TX      | UART RX                    | GPS → Hardware                   |
| RX      | UART TX                    | Hardware → GPS UART              |
| PPS     | Configured via `GpsConfig` | Pulse-per-second timing          |
| FIX     | Not connected              | Hardware status indicator output |
| ENABLED | Internally yied to VIN     | GPS permanently enabled          |

The GPIO numbers supplied to `GpsConfig` represent **BCM GPIO numbers / libgpiod line offsets**, not physical header pin numbers.

The FIX output is intended primarily as a hardware status indicator. The library derives `GpsStatus` from the navigation status reported by the NMEA RMC sentence.

## NMEA Sentences

The GPS receiver continuously transmits NMEA sentences over its UART connection.

The library processes the relevant sentence types individually:
* **RMC** — recommended minimum navigation data, including UTC, date, navigation status, position, speed, course, and magnetic variation.
* **GGA** — GPS fix information, including fix quality, satellites, HDOP, altitude, and geoid separation.
* **GSA** — Satellite and fix selection information.

The sentences are parsed independently, and the resulting values are accumulated into `GpsData`.

A single NMEA sentence contains one sentence type. RMC, GGA, and GSA are separate sentences transmitted by the receiver.

## Pulse-per-Second

The GPS PPS output provides a precise one-pulse-per-second timing signal.

The library configures the PPS GPIO for:
* Input
* Rising-edge event detection

Linux timestamps the PPS rising edge when the GPIO event occurs. The library exposes this timestamp through `PpsEvent`.

The PPS timestamp is not a UTC timestamp by itself. It represents the timestamp assigned by the Linux GPIO subsystem's monotonic clock to the physical PPS edge. An application can correlate this timestamp with the GPS UTC obtained from the NMEA stream.

## GPS Status

The library exposes the receiver state through the `GpsStatus` enum. The status transitions are based on the navigation status reported by the RMC sentence:
* `Initializing` — no valid GPS UTC has been established.
* `NoLock` — The GPS receiver is communicating and providing UTC from its internal RTC, but does not currently have a valid satellite navigation fix.
* `Locked` — the GPS receiver has a valid navigation solution and GPS-synchronized UTC.

**Note:** PPS may be used for timing and does not determine whether the receiver is locked.

## UTC Time

UTC is obtained from the RMC sentence.

Once the receiver has established UTC, it can continue providing time while operating without a satellite fix. In `NoLock`, the receiver's internal RTC provides a continuous UTC estimate until GPS synchronization is restored.

The application can therefore distinguish between:

```text
Initializing
    |
    | First usable GPS time
    v
NoLock <----------------+
    |                   |
    | GPS fix acquired  | GPS fix lost
    v                   |
Locked -----------------+
```

An application can use `GpsStatus` to determine the confidence it places in the reported UTC time.

## GPS Data Model

`GpsData` represents the most recently parsed GPS information.

The data includes information such as:
* GPS Status
* UTC Date
* UTC Time
* Latitude
* Longitude
* Altitude
* Speed
* Course
* 32-point compass heading
* Heading
* Horizontal DOP
* Geoid Separation
* Magnetic Variation

## PPS Event Processing

PPS processing uses the Linux GPIO character-device interface through `libgpiod`. The most recently received PPS event is available via `GetLastPpsEvent()`.

`PpsEvent` contains:
* `valid` — Indicates whether a valid PPS event has been received.
* `sequence` — Increasing PPS event sequence number.
* `timestamp` — Monotonic Kernel timestamp associated with the PPS rising edge.

A `valid == false` event does not indicate an error. It means that no valid PPS event has been received yet. Likewise, if no new PPS event is available during an `Update()` call, the previously received PPS event remains valid as the **last known PPS event**.

The PPS GPIO is configured for rising-edge detection.

`Update()` performs a non-blocking check for pending PPS events. Consequently, the absence of a PPS event is a normal condition and is not treated as a processing failure.

## Building the GPS Object

The library is configured through `GpsConfig`.

A basic initialization looks like this:

```cpp
UltimateGPS::GpsConfig config{
    .serialDevice = "/dev/serial0",
    .baudRate = 9600U,
    .gpioChip = "/dev/gpiochip0",
    .ppsGpio = 24U
};

UltimateGPS::GPS gps(config);

if (!gps.Open())
{
    std::cerr << "Failed to initialize GPS. Error (" << errno << "): "
              << std::strerror(errno) << std::endl;
    return 1;
}
```

`GpsConfig` contains the serial and GPIO configuration required by the library.

The GPIO numbers are supplied through the configuration rather than being hard-coded into the library. This allows the same library to be used with different GPIO assignments.

## Reading GPS Data

Once the GPS has been opened, call `Update()` periodically.

```cpp
while (true)
{
    if (!gps.IsOpen())
    {
        std::cerr << "GPS is not initialized." << std::endl;
        continue;
    }

    if (!gps.Update())
    {
        std::cerr << "GPS update failed. Error (" << errno << "): "
            << std::strerror(errno) << std::endl;
        continue;
    }

    const auto data = gps.GetData();

    std::cout
        << "lat(" << data.latitude
        << ") lng(" << data.longitude
        << ") alt(" << data.altitudeMeters << ")"
        << std::endl;

    std::cout
        << "vel(" << data.speedMetersPerSecond
        << ") cor(" << data.courseDegrees
        << ") hed(" << data.get32PointHeading() << ")"
        << std::endl;

    std::cout
        << std::setfill('0')
        << data.utcYear << "-"
        << std::setw(2) << +data.utcMonth << "-"
        << std::setw(2) << +data.utcDay << "T"
        << std::setw(2) << +data.utcHour << ":"
        << std::setw(2) << +data.utcMinute << ":"
        << std::setw(2) << +data.utcSecond << "Z"
        << std::endl;

    const auto pps = gps.GetLastPpsEvent();

    std::cout
        << "valid(" << pps.valid
        << ") sequence(" << pps.sequence
        << ") timestamp(" << pps.timestamp.count() << ")"
        << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
```

`Update()` performs the library's normal processing cycle:
1. Reads available GPS serial data.
2. Processes complete NMEA sentences.
3. Processes available PPS events.
4. Updates the library's GPS and PPS state.
