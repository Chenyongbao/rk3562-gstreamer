#ifndef SONIX_TOOLKIT_H
#define SONIX_TOOLKIT_H

#include <stdint.h>
#include <stddef.h>
#include "calibData.h"

#ifdef __cplusplus
extern "C" {
#endif

// Flash address for calibration data (matches current project usage)
#define SONIX_CALIB_FLASH_ADDR   0x20000L
#define SONIX_CALIB_SECTOR_SIZE  4096L

// -------- Gain (V4L2) --------
// it is used to get/set light brightness
int usbcamGetLightStatus(int videoIndex, int* status);
int usbcamSetLightStatus(int videoIndex, int brightness);

// -------- Calibration data (Flash) --------
uint8_t usbcamCalib_calc_checksum(const CalibData* data);
int usbcamCalibReadFlash(int videoIndex, long addr, CalibData* out);
int usbcamCalibWriteFlash(int videoIndex, long addr, CalibData data);

// -------- Firmware upgrade --------
// Full-erase burner + restore calib sector
int usbcamUpgradeFwPreserveCalib(int videoIndex, const char* fw_path, long calib_addr);
// Partial erase (only [0, fw_len)) + write, does not touch beyond fw_len
int usbcamUpgradeFwPartial(int videoIndex, const char* fw_path, long calib_addr);

#ifdef __cplusplus
}
#endif

#endif // SONIX_TOOLKIT_H

