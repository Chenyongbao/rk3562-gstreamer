#ifndef __LUOPTDEV_H__
#define __LUOPTDEV_H__

#include "util.h"
#include "ROMData.h"

#include <linux/videodev2.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 36)
#include <linux/uvcvideo.h>
#endif

#define XU_SONIX_SYS_ASIC_RW	      			0x01

#define DF_XU_DATA_SIZE_SF64		64
#define SF_XU64_RW_START_ADDR		0x337
#define	SF_XU64_RW_END_ADDR			0x33B
#define SF_XU64_EU_ADDR				0x335

#pragma pack(push)
#pragma pack(1)
typedef	struct
{
	// Cmd = [7:6], Resvered = [5:0]
	// 0x00: Normal Write, 0x01: Phase Detection, 0x10: Dummy Write
	DWORD dwStartAddr;
	BYTE byCmd;
	BYTE bySFLen;
	DWORD dwEndAddr;
	BYTE bySFReadCmd;
	BYTE bySFOutMoeSel;
} XU6Data;
#pragma pack(pop)


#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 36)
#define UVC_SET_CUR					0x01
#define UVC_GET_CUR					0x81
#define UVCIOC_CTRL_MAP		_IOWR('u', 0x20, struct uvc_xu_control_mapping)
#define UVCIOC_CTRL_QUERY	_IOWR('u', 0x21, struct uvc_xu_control_query)
#else
#define UVCIOC_CTRL_ADD		_IOW('U', 1, struct uvc_xu_control_info)
#define UVCIOC_CTRL_MAP		_IOWR('U', 2, struct uvc_xu_control_mapping)
#define UVCIOC_CTRL_GET		_IOWR('U', 3, struct uvc_xu_control)
#define UVCIOC_CTRL_SET		_IOW('U', 4, struct uvc_xu_control)
#endif

BOOL XU_OpenCamera(char *devPath);
BOOL XU_CloseCamera();

BOOL XU_RestartDevice();

BOOL XU_ReadFromASIC(USHORT addr, BYTE *pValue);
BOOL XU_WriteToASIC(USHORT addr, BYTE value);

BYTE XU_GetUVCExtendUnitID();

BOOL XU_SetAsicArchInfo(DSP_ARCH_TYPE asicArchType);

BOOL XU_GetChipID(LONG idAddr, BYTE *pChipID);
DSP_ROM_TYPE XU_GetChipRomType(BYTE *pChipID, DSP_ARCH_TYPE *pAsicArchType);

BOOL XU_CustomReadFromSensor(BYTE slaveID, USHORT addr, BYTE addrByteNum, USHORT *pData, BYTE dataByteNum, bool pollSCL);
BOOL XU_CustomWriteToSensor(BYTE slaveID, USHORT addr, BYTE addrByteNum, USHORT data, BYTE dataByteNum, bool pollSCL);

BOOL IsSupportI2C64();
BOOL IsSupportXU6();

BOOL SetSFControllerSCK();	// Set SF Controller SCK to 24 MHz
void StopReadWrite_SFXU6();
BOOL SetReadInfo_SFXU6(LONG startAddr, LONG dataSize, LONG bufferSize);
BOOL SetWriteInfo_SFXU6(LONG startAddr, LONG dataSize, LONG bufferSize);
BOOL ReadDataFormFlashXU6(LONG addr, BYTE pData[], BYTE dataLength);
BOOL WriteDataToFlashXU6(LONG addr, BYTE pData[], BYTE dataLength);
BOOL ReadFormSF_XU6(LONG addr, BYTE pData[], LONG len);
BOOL WriteToSF_XU6(LONG addr, BYTE pData[], LONG len);


BOOL SensorTwoWrite285(BYTE slaveID, USHORT addr, BYTE addrByteNum, BYTE* data, BYTE dataByteNum, bool pollSCL);
BOOL SensorTwoWrite(BYTE slaveID, USHORT addr, BYTE addrByteNum, BYTE* data, BYTE dataByteNum, bool pollSCL);
BOOL SensorTwoRead285(BYTE slaveID, USHORT addr, BYTE addrByteNum, BYTE* pData, BYTE dataByteNum, bool pollSCL);
BOOL SensorTwoRead(BYTE slaveID, USHORT addr, BYTE addrByteNum, BYTE* pData, BYTE dataByteNum, bool pollSCL);
BOOL XU_CustomReadFromSensorTwo(BYTE slaveID, USHORT addr, BYTE addrByteNum, BYTE* pData, BYTE dataByteNum, bool pollSCL);
BOOL XU_CustomWriteToSensorTwo(BYTE slaveID, USHORT addr, BYTE addrByteNum, BYTE* data, BYTE dataByteNum, bool pollSCL);

BOOL XU_ReadDataFormFlash(LONG addr, BYTE pData[], BYTE dataLen);
BOOL XU_WriteDataToFlash(LONG addr, BYTE pData[], BYTE dataLen);

BOOL XU_ReadFormSF(LONG addr, BYTE pData[], LONG len);
BOOL XU_WriteToSF(LONG addr, BYTE pData[], LONG len);

BOOL XU_GetAsicRomVersion(BYTE data[]);
BOOL XU_DefGetAsicRomVersion(BYTE data[]);
BOOL XU_ReadFromROM(LONG addr, BYTE data[]);

BOOL XU_EnableAsicRegisterBit(LONG addr, BYTE bit);
BOOL XU_DisableAsicRegisterBit(LONG addr, BYTE bit);

BOOL XU_Read(unsigned char pData[], unsigned int length, BYTE unitID, BYTE cs);
BOOL XU_Write(unsigned char pData[], unsigned int length, BYTE unitID, BYTE cs);

BOOL XU_GetSerialFlashType(SERIAL_FLASH_TYPE *sft, bool check);

BOOL XU_DisableSerialFlashWriteProtect(SERIAL_FLASH_TYPE sft);

BOOL XU_EraseSectorForSerialFlash(LONG addr, SERIAL_FLASH_TYPE sft);
BOOL XU_EraseBlockForSerialFlash(LONG addr, SERIAL_FLASH_TYPE sft);
BOOL XU_SerialFlashErase(SERIAL_FLASH_TYPE sft);

BOOL XU_GetMemType(BYTE *pMemType);
BOOL XU_SFWaitReady();
BOOL XU_SFCMDreadStatus();

BOOL XU_GetParaTableAndCRCAddrFormFW(BYTE *pFW, ULONG* paraTableStartAddr, ULONG* paraTableEndAddr, ULONG* crcAddr);
BOOL XU_GetParaTableAndCRCAddrFormSF(ULONG *paraTableStartAddr, ULONG *paraTableEndAddr, ULONG *crcAddr);
BOOL XU_GetStringSettingFormSF(BYTE* pbyString, DWORD stringSize, DWORD StringOffset, BOOL bIsCRCProtect);

#endif





