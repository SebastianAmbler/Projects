#pragma once
#include <Arduino.h>
#include <SPI.h>

class MPU9250_SPI {
public:
    explicit MPU9250_SPI(uint8_t csPin);
    bool begin();
    uint8_t whoAmI();
    bool readAccelGyro(int16_t accel[3], int16_t gyro[3]);
    // Public wrappers to access MPU registers when needed (e.g. enable AK8963 bypass)
    bool writeMPURegister(uint8_t reg, uint8_t value);
    bool readMPURegister(uint8_t reg, uint8_t* buf, size_t len);

private:
    uint8_t _cs;
    SPISettings _spiSettings;
    void csSelect();
    void csDeselect();
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* buf, size_t len);
};
