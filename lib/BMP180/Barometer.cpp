#include "Barometer.h"

/// @brief Constructor
/// @param filterSize 
Barometer::Barometer(uint8_t filterSize)
    : _filterSize(filterSize), _bufferIndex(0)
{
    // Dinamik bellek ile filtre penceresi oluştur
    _buffer = new float[_filterSize];
    for (uint8_t i = 0; i < _filterSize; i++) {
        _buffer[i] = 0.0;
    }
}
/// @brief Destructor
Barometer::~Barometer() {
    delete[] _buffer;
}

/// @brief Begins the sensor
/// @return Is sensor OK (True, False)
bool Barometer::begin(){
    return bmp.begin();
}
/// @brief Reads Altitude
/// @return (float) Altitude like normal barometer libraries
float Barometer::readMyAltitude() {
    return bmp.readAltitude();
}

float Barometer::readMyPressure(){
    return bmp.readPressure();
}

/// @brief Get altitude but Filtered Data
/// @return Filtered Altitude Data
float Barometer::getMyAltitude() {
    // BMP180'den ham yükseklik verisini al
    float altitude = readMyAltitude();
    // Hareketli ortalama filtresini uygula
    return movingAverage(altitude);
}

float Barometer::getMyPressure(){
    return bmp.readPressure();
}

/// @brief For getting current Altitude calculations ( it returns filtered data )
/// @param pressure 
/// @return Current Altitude not from sea level
float Barometer::getAltitudeFromHomePoint(float pressure){
    float altitude = bmp.readAltitude(pressure);
    return movingAverage(altitude);
}

/// @brief For Calculating barometer offset
/// @return Barometer offset 
float Barometer::calibrateGyro(){
    const int numReadings = 10;
    long totalPressure;

    for(int i = 0; i < numReadings; i++){
        totalPressure = bmp.readAltitude();
        delay(5);
    }

    float averagePressure = totalPressure / (float)numReadings;
    float pressureOffset = 101325 - averagePressure;
    return pressureOffset;
}
/// @brief Filter Function
/// @param newValue 
/// @return Filtered Data
float Barometer::movingAverage(float newValue) {
    _buffer[_bufferIndex] = newValue;
    _bufferIndex = (_bufferIndex + 1) % _filterSize;
    
    float sum = 0;
    for (uint8_t i = 0; i < _filterSize; i++) {
        sum += _buffer[i];
    }
    return sum / _filterSize;
}