#include "calib_eeprom.h"
#include <iostream>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

using namespace std;

uint8_t CalibEEPROM::calcChecksum(const uint8_t* data, int len)
{
    if (!data || len <= 0) return 0;
    return calib_xor_checksum(data, (size_t)len);
}

CalibEEPROM::CalibEEPROM(int i2cBus, uint8_t deviceAddr)
    : m_i2cBus(i2cBus)
    , m_deviceAddr(deviceAddr)
    , m_fd(-1)
{
}

CalibEEPROM::~CalibEEPROM()
{
    close();
}

bool CalibEEPROM::open()
{
    if (m_fd >= 0) {
        return true;
    }
    
    char devPath[32];
    snprintf(devPath, sizeof(devPath), "/dev/i2c-%d", m_i2cBus);
    
    m_fd = ::open(devPath, O_RDWR);
    if (m_fd < 0) {
        cerr << "CalibEEPROM: Failed to open " << devPath 
             << " (" << strerror(errno) << ")" << endl;
        return false;
    }
    
    if (ioctl(m_fd, I2C_SLAVE, m_deviceAddr) < 0) {
        cerr << "CalibEEPROM: Failed to set slave addr 0x" 
             << hex << (int)m_deviceAddr << dec << endl;
        ::close(m_fd);
        m_fd = -1;
        return false;
    }
    
    return true;
}

void CalibEEPROM::close()
{
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

bool CalibEEPROM::waitForWriteComplete()
{
    usleep(WRITE_TIME_MS * 1000);
    return true;
}

bool CalibEEPROM::writeBytes(uint16_t memAddr, const uint8_t* data, int len)
{
    if (!isOpen() || data == nullptr || len <= 0) {
        return false;
    }
    
    int written = 0;
    while (written < len) {
        int pageOffset = (memAddr + written) % PAGE_SIZE;
        int bytesInPage = PAGE_SIZE - pageOffset;
        int bytesToWrite = min(bytesInPage, len - written);
        
        uint8_t buf[PAGE_SIZE + 2];
        uint16_t addr = memAddr + written;
        buf[0] = (addr >> 8) & 0xFF;
        buf[1] = addr & 0xFF;
        memcpy(buf + 2, data + written, bytesToWrite);
        
        if (::write(m_fd, buf, bytesToWrite + 2) != bytesToWrite + 2) {
            return false;
        }
        
        waitForWriteComplete();
        written += bytesToWrite;
    }
    
    return true;
}

bool CalibEEPROM::readBytes(uint16_t memAddr, uint8_t* data, int len)
{
    if (!isOpen() || data == nullptr || len <= 0) {
        return false;
    }
    
    uint8_t addrBuf[2];
    addrBuf[0] = (memAddr >> 8) & 0xFF;
    addrBuf[1] = memAddr & 0xFF;
    
    if (::write(m_fd, addrBuf, 2) != 2) {
        return false;
    }
    
    if (::read(m_fd, data, len) != len) {
        return false;
    }
    
    return true;
}

bool CalibEEPROM::write(const CalibData& data)
{
    if (!open()) {
        return false;
    }
    
    CalibData calibData = data;
    int dataLen = sizeof(CalibData) - sizeof(calibData.checksum);
    calibData.checksum = calcChecksum(reinterpret_cast<uint8_t*>(&calibData), dataLen);
    
    bool success = writeBytes(0, reinterpret_cast<uint8_t*>(&calibData), sizeof(CalibData));
    
    close();
    
    return success;
}

bool CalibEEPROM::read(CalibData& data)
{
    if (!open()) {
        return false;
    }
    
    bool success = readBytes(0, reinterpret_cast<uint8_t*>(&data), sizeof(CalibData));
    
    close();
    
    if (!success) {
        return false;
    }
    
    // Verify checksum
    int dataLen = sizeof(CalibData) - sizeof(data.checksum);
    uint8_t calcCk = calcChecksum(reinterpret_cast<uint8_t*>(&data), dataLen);
    
    if (calcCk != data.checksum) {
        cerr << "CalibEEPROM: Checksum mismatch" << endl;
        return false;
    }
    
    return true;
}
