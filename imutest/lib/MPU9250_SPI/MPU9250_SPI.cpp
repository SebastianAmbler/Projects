#include "MPU9250_SPI.h"

// MPU9250 registers
static const uint8_t REG_WHO_AM_I = 0x75;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

MPU9250_SPI::MPU9250_SPI(uint8_t csPin)
    : _cs(csPin), _spiSettings(1000000, MSBFIRST, SPI_MODE0) {}

void MPU9250_SPI::csSelect() {
    digitalWrite(_cs, LOW);
}

void MPU9250_SPI::csDeselect() {
    digitalWrite(_cs, HIGH);
}

bool MPU9250_SPI::writeRegister(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(_spiSettings);
    csSelect();
    SPI.transfer(reg & 0x7F); // write: MSB = 0
    SPI.transfer(value);
    csDeselect();
    SPI.endTransaction();
    return true;
}

bool MPU9250_SPI::readRegisters(uint8_t reg, uint8_t* buf, size_t len) {
    SPI.beginTransaction(_spiSettings);
    csSelect();
    SPI.transfer(reg | 0x80); // read: MSB = 1
    for (size_t i = 0; i < len; ++i) {
        buf[i] = SPI.transfer(0x00);
    }
    csDeselect();
    SPI.endTransaction();
    return true;
}

bool MPU9250_SPI::begin() {
    pinMode(_cs, OUTPUT);
    csDeselect();
    SPI.begin();

    // Reset device
    writeRegister(REG_PWR_MGMT_1, 0x80);
    delay(100);

    // Wake up and set clock source
    writeRegister(REG_PWR_MGMT_1, 0x01);
    delay(10);

    // Sample rate divider
    writeRegister(REG_SMPLRT_DIV, 0x00);

    // Config: DLPF
    writeRegister(REG_CONFIG, 0x03);

    // Gyro full scale ±250 dps
    writeRegister(REG_GYRO_CONFIG, 0x00);

    // Accel full scale ±2g
    writeRegister(REG_ACCEL_CONFIG, 0x00);

    // Verify WHO_AM_I
    uint8_t id = whoAmI();
    return (id == 0x71 || id == 0x73); // 0x71 typical for MPU9250, some variants 0x73
}

uint8_t MPU9250_SPI::whoAmI() {
    uint8_t v = 0;
    readRegisters(REG_WHO_AM_I, &v, 1);
    return v;
}

bool MPU9250_SPI::readAccelGyro(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buf[14];
    if (!readRegisters(REG_ACCEL_XOUT_H, buf, sizeof(buf))) return false;

    // Accel
    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);

    // Temp (buf[6], buf[7]) ignored here

    // Gyro
    gyro[0] = (int16_t)((buf[8] << 8) | buf[9]);
    gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
    gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);

    return true;
}

// Public wrappers to expose register access
bool MPU9250_SPI::writeMPURegister(uint8_t reg, uint8_t value) {
    return writeRegister(reg, value);
}

bool MPU9250_SPI::readMPURegister(uint8_t reg, uint8_t* buf, size_t len) {
    return readRegisters(reg, buf, len);
}
