#ifndef CALIB_EEPROM_H
#define CALIB_EEPROM_H

#include "calibData.h"

/**
 * @brief EEPROM read/write class for camera calibration data
 * 
 * This class provides a simple interface for storing and retrieving
 * camera calibration parameters to/from I2C EEPROM (BL24C64F).
 */
class CalibEEPROM {
public:
    /**
     * @brief Constructor
     * @param i2cBus I2C bus number (e.g., 4 for /dev/i2c-4)
     * @param deviceAddr EEPROM I2C address (default 0x50 for BL24C64F)
     */
    CalibEEPROM(int i2cBus = 4, uint8_t deviceAddr = 0x50);
    
    ~CalibEEPROM();

    /**
     * @brief Write calibration data to EEPROM
     * @param data Calibration data structure (checksum will be auto-calculated)
     * @return true on success, false on failure
     */
    bool write(const CalibData& data);

    /**
     * @brief Read calibration data from EEPROM
     * @param data Output calibration data structure
     * @return true on success with valid checksum, false on failure
     */
    bool read(CalibData& data);

    /**
     * @brief Calculate XOR checksum
     */
    static uint8_t calcChecksum(const uint8_t* data, int len);

private:
    bool open();
    void close();
    bool isOpen() const { return m_fd >= 0; }
    bool writeBytes(uint16_t memAddr, const uint8_t* data, int len);
    bool readBytes(uint16_t memAddr, uint8_t* data, int len);
    bool waitForWriteComplete();

    int m_i2cBus;
    uint8_t m_deviceAddr;
    int m_fd;
    
    static const int PAGE_SIZE = 32;
    static const int WRITE_TIME_MS = 5;
};

#endif // CALIB_EEPROM_H
