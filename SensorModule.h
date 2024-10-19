#pragma once

#include <stdint.h>

// Enum to define read modes for sensors
enum class ReadMode : uint8_t
{
    Off,
    On,
    OnAndOff,
    OddEven,
    OddEvenAndOff,
    Manual
};

// Enum to define types of sensors

enum class Type : uint8_t
{
    Undefined,
    RC,
    Analog
};

// Default timeout for RC sensors (in microseconds).
const uint16_t DefaultTimeout = 2500;

// The maximum number of sensors supported by an instance of this class.
const uint8_t MaxSensors = 31;

class Sensors
{
public:
    Sensors() = default;
    ~Sensors();

    // Call this function to set up RC-type sensors.
    void setTypeRC();

    // Call this function to set up A-type sensors.
    void setTypeAnalog();

    // Sets the sensor pins.
    //
    // pins A pointer to an array containing the Arduino pins that the sensors are connected to.
    //
    // sensorCount The number of sensors, which should match the length of the pins array.
    void setSensorPins(const uint8_t *pins, uint8_t sensorCount);

    // Calibrate the sensors based on the specified read mode
    void calibrate(ReadMode mode = ReadMode::On);

    // Resets all calibration that has been done.
    void resetCalibration();

    // Reads the raw sensor values into an array.
    //
    // sensorValues A pointer to an array in which to store the
    // raw sensor readings. There **MUST** be space in the array for as many
    // values as there were sensors specified in setSensorPins().
    //
    // mode The emitter behavior during the read, as a member of the
    // ::ReadMode enum. The default is ReadMode::On.
    void read(uint16_t *sensorValues, ReadMode mode = ReadMode::On);

    // Reads the sensors and provides calibrated values between 0 and 1000.
    //
    //  sensorValues A pointer to an array in which to store the
    //  calibrated sensor readings.  There **MUST** be space in the array for
    //  as many values as there were sensors specified in setSensorPins().
    //
    //  mode The emitter behavior during the read, as a member of the
    //  ::ReadMode enum. The default is ReadMode::On. Manual emitter control with ReadMode::Manual is not supported.
    void readCalibrated(uint16_t *sensorValues, ReadMode mode = ReadMode::On);

    // Reads the sensors, provides calibrated values, and returns an estimated position of a black line under the sensors.
    //
    // This function is designed to detect a black (or dark-colored) line on a white (or light-colored) background.
    // It uses a weighted average of sensor readings to calculate the position of the line.
    //
    // sensorValues
    //     A pointer to an array where the calibrated sensor readings will be stored.
    //     There **MUST** be enough space in the array for the number of sensors specified in setSensorPins().
    //
    // mode
    //     The emitter behavior during the read. This should be a member of the ::QTRReadMode enum.
    //     The default mode is QTRReadMode::On. Note: Manual emitter control (QTRReadMode::Manual) is not supported.
    //
    // return
    //     An estimate of the position of the black line under the sensors.
    //     The return value is calculated as follows:
    //     - A return value of 0 indicates the line is directly below sensor 0.
    //     - A return value of 1000 indicates the line is directly below sensor 1.
    //     - A return value of 2000 indicates the line is directly below sensor 2, and so on.
    //     - Intermediate values indicate the line is positioned between two sensors.
    //
    // The calculation is done using the formula:
    // \f[
    // {(0 \times v_0) + (1000 \times v_1) + (2000 \times v_2) + \cdots
    // \over
    // v_0 + v_1 + v_2 + \cdots}
    // \f]
    //
    // This function remembers the last position of the line, allowing it to indicate the direction needed to reacquire the line
    // if it is lost to the left or right. For example, if sensor 4 is the rightmost sensor and you move completely off the line
    // to the left, the function will still return 4000.
    uint16_t readLineBlack(uint16_t *sensorValues, ReadMode mode = ReadMode::On)
    {
        return readLinePrivate(sensorValues, mode, false);
    }

    // Stores sensor calibration data.
    struct CalibrationData
    {
        /// Whether array pointers have been allocated and initialized.
        bool initialized = false;
        /// Lowest readings seen during calibration.
        uint16_t *minimum = nullptr;
        /// Highest readings seen during calibration.
        uint16_t *maximum = nullptr;
    };

    // Data from calibrating with emitters on.
    CalibrationData calibrationOn;

    // Data from calibrating with emitters off.
    CalibrationData calibrationOff;

private:
    // Handles the actual calibration, including (re)allocating and
    // initializing the storage for the calibration values if necessary.
    void calibrateOnOrOff(CalibrationData &calibration, ReadMode mode);

    void readPrivate(uint16_t *sensorValues, uint8_t start = 0, uint8_t step = 1);

    uint16_t readLinePrivate(uint16_t *sensorValues, ReadMode mode, bool invertReadings);

    Type _type = Type::Undefined;

    uint8_t *_sensorPins = nullptr;
    uint8_t _sensorCount = 0;

    uint16_t _timeout = DefaultTimeout;  // only used for RC sensors
    uint16_t _maxValue = DefaultTimeout; // the maximum value returned by readPrivate()
    uint8_t _samplesPerSensor = 4;       // only used for analog sensors

    uint16_t _lastPosition = 0;
};
