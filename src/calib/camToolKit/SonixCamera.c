#include "SonixCamera.h"
#include "XUOptDev.h"
#include "util.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ROMData.h"
BOOL camera_init = false;
extern unsigned int uiRomID;

BYTE GetByteLengthFormAddr(LONG addr)
{
	if (addr <= 0xFF)
		return 1;
	else if (addr <= 0xFFFF)
		return 2;
	else if (addr <= 0xFFFFFF)
		return 3;
	else
		return 4;
}

bool SonixCam_Init(int videoIndex)
{
    if(camera_init)
        return true;

	char devPath[300] = {0};
	sprintf(devPath, "/dev/video%d", videoIndex);
	if(!XU_OpenCamera(devPath))
	{
		fprintf(stderr, "Open video device failed\n");
		return false;
	}

	camera_init = true;
    return true;
}

bool SonixCam_UnInit()
{
	if (!camera_init)
		return false;

	if(!XU_CloseCamera())
	{
		return false;
	}
	camera_init = false;
    return true;
}

bool SonixCam_RestartDevice()
{
	if (!camera_init)
		return false;

	return XU_RestartDevice();
}


bool SonixCam_AsicRegisterRead(unsigned short addr, unsigned char pData[], long len)
{
	if (!camera_init)
		return false;
	
	BYTE data = 0;
	USHORT startAddr = addr;
	LONG i = 0;
	for (i = 0; i < len; i++)
	{
		if (true != XU_ReadFromASIC((USHORT)startAddr++, &data))
			return false;
		memcpy(pData + i, &data, 1);
	}
	return true;
}

bool SonixCam_AsicRegisterWrite(unsigned short addr, unsigned char pData[], long len)
{
	if (!camera_init)
		return false;
	
	BYTE data = 0;
	LONG startAddr = addr;
	LONG i = 0;
	for (i = 0; i < len; i++)
	{
		data = pData[i];
		if (true != XU_WriteToASIC((USHORT)startAddr++, data))
			return false;
	}
	return true;
}

bool SonixCam_GetSerialFlashType(SERIAL_FLASH_TYPE *sft, bool check)
{
	if (!camera_init)
		return false;

	return XU_GetSerialFlashType(sft, check);
}

bool  SonixCam_GetFwVersion(unsigned char pData[], long len, BOOL bNormalExport)
{
	if (!camera_init)
		return false;

	BYTE RomValue[10] = { 0 };
	BOOL hr = XU_GetAsicRomVersion(RomValue);

	BYTE chipID;
	DSP_ARCH_TYPE asicArchType;
	if (DAT_UNKNOW == XU_GetChipRomType(&chipID, &asicArchType))
		return false;
	
	BYTE FlashCodeVer[56] = { 0 };
	BYTE CusVer[31];
	BYTE DayVer[31];
	BYTE Customer[31];

	switch (chipID){
	case 0x15:
		XU_ReadFormSF(0x3E00, Customer + 2, 10);
		XU_ReadFormSF(0x3E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x3E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x16:
		XU_ReadFormSF(0x0FD0, Customer + 2, 10);
		XU_ReadFormSF(0x0FD0 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x0FD0 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x0148 + 11, FlashCodeVer, 5);
		break;
	case 0x20:
		XU_ReadFormSF(0x5E00, Customer + 2, 10);
		XU_ReadFormSF(0x5E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x5E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x0 + 11, FlashCodeVer, 4);
		break;
	case 0x25:
		XU_ReadFormSF(0x5E00, Customer + 2, 10);
		XU_ReadFormSF(0x5E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x5E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x30:
		XU_ReadFormSF(0x1E00, Customer + 2, 10);
		XU_ReadFormSF(0x1E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x1E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
	case 0x31:
		if (true != hr){
			return false;
		}
		if (RomValue[5] == 1){
			XU_ReadFormSF(0x1E00, Customer + 2, 10);
			XU_ReadFormSF(0x1E00 + 10, CusVer + 2, 10);
			XU_ReadFormSF(0x1E00 + 20, DayVer + 2, 10);
			XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		}
		else if (RomValue[5] == 2){
			XU_ReadFormSF(0x6000 + 0x1E0, Customer + 2, 10);
			XU_ReadFormSF(0x6000 + 0x1E00 + 10, CusVer + 2, 10);
			XU_ReadFormSF(0x6000 + 0x1E00 + 20, DayVer + 2, 10);
			XU_ReadFormSF(0x6000 + 0x01e8 + 11, FlashCodeVer, 5);
		}
		else{
			return false;
		}

		break;
	case 0x32:
		if (true != hr){
			return false;
		}
		if (RomValue[5] == 1){
			XU_ReadFormSF(0x0FD0, Customer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 10, CusVer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 20, DayVer + 2, 10);
			XU_ReadFormSF(0x0148 + 11, FlashCodeVer, 5);
		}
		else if (RomValue[5] == 2){
			XU_ReadFormSF(0x0FD0, Customer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 18, CusVer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 28, DayVer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 10, FlashCodeVer, 5);
		}
		else{
			return false;
		}
		break;
	case 0x36:
		XU_ReadFormSF(0x3E00, Customer + 2, 10);
		XU_ReadFormSF(0x3E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x3E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x50:
		XU_ReadFormSF(0x5E00, Customer + 2, 10);
		XU_ReadFormSF(0x5E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x5E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x56:
		XU_ReadFormSF(0x3E00, Customer + 2, 10);
		XU_ReadFormSF(0x3E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x3E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x70:
		XU_ReadFormSF(0x3E05, Customer + 2, 10);
		XU_ReadFormSF(0x3E0F, CusVer + 2, 10);
		XU_ReadFormSF(0x3E19, DayVer + 2, 10);
		XU_ReadFormSF(0x3E00, FlashCodeVer, 5);
		break;
	case 0x71:
		XU_ReadFormSF(0x3E05, Customer + 2, 10);
		XU_ReadFormSF(0x3E0F, CusVer + 2, 10);
		XU_ReadFormSF(0x3E19, DayVer + 2, 10);
		XU_ReadFormSF(0x3E00, FlashCodeVer, 5);
		break;
	case 0x75:
		if (RomValue[5] == 1){
			XU_ReadFormSF(0x2000, Customer + 2, 10);
			XU_ReadFormSF(0x2012, CusVer + 2, 10);
			XU_ReadFormSF(0x201C, DayVer + 2, 10);
			XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		}
		else{
			return false;
		}

		break;
	case 0x76: //128K
		if (RomValue[5] == 1){
			XU_ReadFormSF(0x3FD0, Customer + 2, 10);
			XU_ReadFormSF(0x3FD0 + 10, CusVer + 2, 10);
			XU_ReadFormSF(0x3FD0 + 20, DayVer + 2, 10);
			XU_ReadFormSF(0x0148 + 11, FlashCodeVer, 5);
		}
		else{
			return false;
		}
		break;
	case 0x83: //128K
	case 0x85: //128K
	case 0x86: //128K
		XU_ReadFormSF(0x2000, Customer + 2, 10);
		XU_ReadFormSF(0x2012, CusVer + 2, 10);
		XU_ReadFormSF(0x201C, DayVer + 2, 10);
		XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		break;
	case 0x89:	
	case 0x90:
		XU_ReadFormSF(0x2000, Customer + 2, 10);
		XU_ReadFormSF(0x2012, CusVer + 2, 10);
		XU_ReadFormSF(0x201D, DayVer + 2, 10);
		XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		break;
	case 0x92: //128K
		XU_ReadFormSF(0x2000, Customer + 2, 10);
		XU_ReadFormSF(0x2012, CusVer + 2, 10);
		XU_ReadFormSF(0x201C, DayVer + 2, 10);
		XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		break;
	default:
		return false;
		break;
	}

	//220
	if (chipID == 0x20){
		FlashCodeVer[40] = '\0';
	}
	else{
		FlashCodeVer[41] = '\0';
	}

if(bNormalExport){
		memcpy(FlashCodeVer + 5, CusVer + 2, 10);
		memcpy(FlashCodeVer + 15, Customer + 2, 10);
		memcpy(FlashCodeVer + 25, DayVer + 2, 10);
	}else{
		/* 第二种组合方式	*/
		BYTE cFLashCodeVer2 = FlashCodeVer[4];
		FlashCodeVer[4] = '-';
		int index = 0;
		BYTE *pFWVersion = FlashCodeVer;
		pFWVersion += 5;
		int i = 0;
		for (i = 0; i<10; i++)
		{
			if (CusVer[i + 2] != 0x2d)
			{
				*(pFWVersion++) = CusVer[i + 2];
			}
		}
		if ( 0 != memcmp(RomValue, "220R", 4))
		{
			*(pFWVersion++) = '-';
			*(pFWVersion++) = cFLashCodeVer2;
		}
		char cDayVer1[7] = {0};
		char cDayVer2[3] = {0};
		char cDayVer3[3] = {0};
		memcpy(cDayVer1, DayVer + 2, 6);
		memcpy(cDayVer2, DayVer + 8, 2);
		memcpy(cDayVer3, DayVer + 10, 2);

		*(pFWVersion++) = '-';
		*(pFWVersion++) = 'V';
		memcpy(pFWVersion, cDayVer1, 6);
		pFWVersion += 6;
		*(pFWVersion++) = '_';
		memcpy(pFWVersion, cDayVer2, 2);
		pFWVersion += 2;
		*(pFWVersion++) = '_';
		memcpy(pFWVersion, cDayVer3, 2);
		pFWVersion += 2;
		*(pFWVersion++) = '-';

		for (i = 0; i < 10; i++)
		{
			if (Customer[i + 2] != 0x2d)
			{
				*(pFWVersion++) = Customer[i + 2];
			}
		}
	}

	char Temp[56] = { 0 };
	memcpy(Temp, FlashCodeVer, 56);
	memset(pData, 0, len);
	memcpy(pData, Temp, (len > 56) ? 56 : len);

	return true;
}

bool  SonixCam_GetFwVersionFromFile(unsigned char pFwFile[], unsigned char pData[], long len, BOOL bNormalExport)
{
	if (!camera_init)
		return false;

	BYTE RomValue[10] = { 0 };
	BOOL hr = XU_GetAsicRomVersion(RomValue);

	BYTE chipID;
	DSP_ARCH_TYPE asicArchType;
	if (DAT_UNKNOW == XU_GetChipRomType(&chipID, &asicArchType))
		return false;
	
	BYTE FlashCodeVer[56] = { 0 };
	BYTE CusVer[31];
	BYTE DayVer[31];
	BYTE Customer[31];

	switch (chipID){
	case 0x15:
		memcpy(Customer + 2, pFwFile + 0x3E00, 10);
		memcpy(CusVer + 2, pFwFile + 0x3E00 + 10, 10);
		memcpy(DayVer + 2, pFwFile + 0x3E00 + 20, 10);
		memcpy(FlashCodeVer, pFwFile + 0x01e8 + 11, 5);
		break;
	case 0x16:
		memcpy(Customer + 2, pFwFile + 0x0FD0, 10);
		memcpy(CusVer + 2, pFwFile + 0x0FD0 + 10, 10);
		memcpy(DayVer + 2, pFwFile + 0x0FD0 + 20, 10);
		memcpy(FlashCodeVer, pFwFile + 0x0148 + 11, 5);
		break;
	case 0x20:
		memcpy(Customer + 2, pFwFile + 0x5E00, 10);
		memcpy(CusVer + 2, pFwFile + 0x5E00 + 10, 10);
		memcpy(DayVer + 2, pFwFile + 0x5E00 + 20, 10);
		memcpy(FlashCodeVer, pFwFile + 0x0 + 11, 4);
		break;
	case 0x25:
		memcpy(Customer + 2, pFwFile + 0x5E00, 10);
		memcpy(CusVer + 2, pFwFile + 0x5E00 + 10, 10);
		memcpy(DayVer + 2, pFwFile + 0x5E00 + 20, 10);
		memcpy(FlashCodeVer, pFwFile + 0x01e8 + 11, 5);
		break;
	case 0x30:
		memcpy(Customer + 2, pFwFile + 0x1E00, 10);
		memcpy(CusVer + 2, pFwFile + 0x1E00 + 10, 10);
		memcpy(DayVer + 2, pFwFile + 0x1E00 + 20, 10);
		memcpy(FlashCodeVer, pFwFile + 0x01e8 + 11, 5);
	case 0x31:
		if (!hr){
			return false;
		}
		if (RomValue[5] == 1){
			memcpy(Customer + 2, pFwFile + 0x1E00, 10);
			memcpy(CusVer + 2, pFwFile + 0x1E00 + 10, 10);
			memcpy(DayVer + 2, pFwFile + 0x1E00 + 20, 10);
			memcpy(FlashCodeVer, pFwFile + 0x01e8 + 11, 5);
		}
		else if (RomValue[5] == 2){
			memcpy(Customer + 2, pFwFile + 0x6000 + 0x1E00, 10);
			memcpy(CusVer + 2, pFwFile + 0x6000 + 0x1E00 + 10, 10);
			memcpy(DayVer + 2, pFwFile + +0x1E00 + 0x1E00 + 20, 10);
			memcpy(FlashCodeVer, pFwFile + 0x6000 + 0x01e8 + 11, 5);
		}
		else{
			return false;
		}

		break;
	case 0x32:
		if (!hr){
			return false;
		}
		if (RomValue[5] == 1){
			memcpy(Customer + 2, pFwFile + 0x0FD0, 10);
			memcpy(CusVer + 2, pFwFile + 0x0FD0 + 10, 10);
			memcpy(DayVer + 2, pFwFile + 0x0FD0 + 20, 10);
			memcpy(FlashCodeVer, pFwFile + 0x0148 + 11, 5);
		}
		else if (RomValue[5] == 2){
			memcpy(Customer + 2, pFwFile + 0x0FD0, 10);
			memcpy(CusVer + 2, pFwFile + 0x0FD0 + 18, 10);
			memcpy(DayVer + 2, pFwFile + 0x0FD0 + 28, 10);
			memcpy(FlashCodeVer, pFwFile + 0x0FD0 + 10, 5);
		}
		else{
			return false;
		}
		break;
	case 0x36:
		memcpy(Customer + 2, pFwFile + 0x3E00, 10);
		memcpy(CusVer + 2, pFwFile + 0x3E00 + 10, 10);
		memcpy(DayVer + 2, pFwFile + 0x3E00 + 20, 10);
		memcpy(FlashCodeVer, pFwFile + 0x01e8 + 11, 5);
		break;
	case 0x50:
		memcpy(Customer + 2, pFwFile + 0x5E00, 10);
		memcpy(CusVer + 2, pFwFile + 0x5E00 + 10, 10);
		memcpy(DayVer + 2, pFwFile + 0x5E00 + 20, 10);
		memcpy(FlashCodeVer, pFwFile + 0x01e8 + 11, 5);
		break;
	case 0x56:
		memcpy(Customer + 2, pFwFile + 0x3E00, 10);
		memcpy(CusVer + 2, pFwFile + 0x3E00 + 10, 10);
		memcpy(DayVer + 2, pFwFile + 0x3E00 + 20, 10);
		memcpy(FlashCodeVer, pFwFile + 0x01e8 + 11, 5);
		break;
	case 0x70:
		memcpy(Customer + 2, pFwFile + 0x3E05, 10);
		memcpy(CusVer + 2, pFwFile + 0x3E0F, 10);
		memcpy(DayVer + 2, pFwFile + 0x3E19, 10);
		memcpy(FlashCodeVer, pFwFile + 0x3E00, 5);
		break;
	case 0x71:
		memcpy(Customer + 2, pFwFile + 0x3E05, 10);
		memcpy(CusVer + 2, pFwFile + 0x3E0F, 10);
		memcpy(DayVer + 2, pFwFile + 0x3E19, 10);
		memcpy(FlashCodeVer, pFwFile + 0x3E00, 5);
		break;
	case 0x75:
		if (RomValue[5] == 1){
			memcpy(Customer + 2, pFwFile + 0x2000, 10);
			memcpy(CusVer + 2, pFwFile + 0x2012, 10);
			memcpy(DayVer + 2, pFwFile + 0x201C, 10);
			memcpy(FlashCodeVer, pFwFile + 0x200A, 5);
		}
		else{
			return false;
		}

		break;
	case 0x76: //128K
		if (RomValue[5] == 1){
			memcpy(Customer + 2, pFwFile + 0x3FD0, 10);
			memcpy(CusVer + 2, pFwFile + 0x3FD0 + 10, 10);
			memcpy(DayVer + 2, pFwFile + 0x3FD0 + 20, 10);
			memcpy(FlashCodeVer, pFwFile + 0x0148 + 11, 5);
		}
		else{
			return false;
		}
		break;
	case 0x83: //128K
	case 0x85: //128K
	case 0x86: //128K
		memcpy(Customer + 2, pFwFile + 0x2000, 10);
		memcpy(CusVer + 2, pFwFile + 0x2012, 10);
		memcpy(DayVer + 2, pFwFile + 0x201C, 10);
		memcpy(FlashCodeVer, pFwFile + 0x200A, 5);
		break;
	case 0x89:
	case 0x90:
		memcpy(Customer + 2, pFwFile + 0x2000, 10);
		memcpy(CusVer + 2, pFwFile + 0x2012, 10);
		memcpy(DayVer + 2, pFwFile + 0x201D, 10);
		memcpy(FlashCodeVer, pFwFile + 0x200A, 5);
		break;
	case 0x92: //128K
		memcpy(Customer + 2, pFwFile + 0x2000, 10);
		memcpy(CusVer + 2, pFwFile + 0x2012, 10);
		memcpy(DayVer + 2, pFwFile + 0x201C, 10);
		memcpy(FlashCodeVer, pFwFile + 0x200A, 5);
		break;
	default:
		return false;
		break;
	}
	//220
	if (chipID == 0x20){
		FlashCodeVer[40] = '\0';
	}
	else{
		FlashCodeVer[41] = '\0';
	}

if(bNormalExport){
		memcpy(FlashCodeVer + 5, CusVer + 2, 10);
		memcpy(FlashCodeVer + 15, Customer + 2, 10);
		memcpy(FlashCodeVer + 25, DayVer + 2, 10);
	}else{
		/* 第二种组合方式	*/
		BYTE cFLashCodeVer2 = FlashCodeVer[4];
		FlashCodeVer[4] = '-';
		int index = 0;
		BYTE *pFWVersion = FlashCodeVer;
		pFWVersion += 5;
		int i = 0;
		for (i = 0; i<10; i++)
		{
			if (CusVer[i + 2] != 0x2d)
			{
				*(pFWVersion++) = CusVer[i + 2];
			}
		}
		if ( 0 != memcmp(RomValue, "220R", 4))
		{
			*(pFWVersion++) = '-';
			*(pFWVersion++) = cFLashCodeVer2;
		}
		char cDayVer1[7] = {0};
		char cDayVer2[3] = {0};
		char cDayVer3[3] = {0};
		memcpy(cDayVer1, DayVer + 2, 6);
		memcpy(cDayVer2, DayVer + 8, 2);
		memcpy(cDayVer3, DayVer + 10, 2);

		*(pFWVersion++) = '-';
		*(pFWVersion++) = 'V';
		memcpy(pFWVersion, cDayVer1, 6);
		pFWVersion += 6;
		*(pFWVersion++) = '_';
		memcpy(pFWVersion, cDayVer2, 2);
		pFWVersion += 2;
		*(pFWVersion++) = '_';
		memcpy(pFWVersion, cDayVer3, 2);
		pFWVersion += 2;
		*(pFWVersion++) = '-';

		for (i = 0; i < 10; i++)
		{
			if (Customer[i + 2] != 0x2d)
			{
				*(pFWVersion++) = Customer[i + 2];
			}
		}
	}

	char Temp[56] = { 0 };
	memcpy(Temp, FlashCodeVer, 56);
	memset(pData, 0, len);
	memcpy(pData, Temp, (len > 56) ? 56 : len);

	return true;
}



bool  SonixCam_GetFwVersionEx(DSP_ROM_TYPE romType, unsigned char pData[], long len, BOOL bNormalExport)
{
	if (!camera_init)
		return false;

	BYTE RomValue[10] = { 0 };
	BOOL hr = XU_GetAsicRomVersion(RomValue);

	BYTE chipID;
	DSP_ARCH_TYPE asicArchType;
	if (DRT_Unknow == XU_GetChipRomType(&chipID, &asicArchType))
		return false;
	
	BYTE FlashCodeVer[56] = { 0 };
	BYTE CusVer[31];
	BYTE DayVer[31];
	BYTE Customer[31];

	switch (chipID){
	case 0x15:
		XU_ReadFormSF(0x3E00, Customer + 2, 10);
		XU_ReadFormSF(0x3E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x3E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x16:
		XU_ReadFormSF(0x0FD0, Customer + 2, 10);
		XU_ReadFormSF(0x0FD0 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x0FD0 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x0148 + 11, FlashCodeVer, 5);
		break;
	case 0x20:
		XU_ReadFormSF(0x5E00, Customer + 2, 10);
		XU_ReadFormSF(0x5E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x5E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x0 + 11, FlashCodeVer, 4);
		break;
	case 0x25:
		XU_ReadFormSF(0x5E00, Customer + 2, 10);
		XU_ReadFormSF(0x5E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x5E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x30:
		XU_ReadFormSF(0x1E00, Customer + 2, 10);
		XU_ReadFormSF(0x1E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x1E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
	case 0x31:
		if (true != hr){
			return false;
		}
		if (RomValue[5] == 1){
			XU_ReadFormSF(0x1E00, Customer + 2, 10);
			XU_ReadFormSF(0x1E00 + 10, CusVer + 2, 10);
			XU_ReadFormSF(0x1E00 + 20, DayVer + 2, 10);
			XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		}
		else if (RomValue[5] == 2){
			XU_ReadFormSF(0x6000 + 0x1E0, Customer + 2, 10);
			XU_ReadFormSF(0x6000 + 0x1E00 + 10, CusVer + 2, 10);
			XU_ReadFormSF(0x6000 + 0x1E00 + 20, DayVer + 2, 10);
			XU_ReadFormSF(0x6000 + 0x01e8 + 11, FlashCodeVer, 5);
		}
		else{
			return false;
		}

		break;
	case 0x32:
		if (true != hr){
			return false;
		}
		if (RomValue[5] == 1){
			XU_ReadFormSF(0x0FD0, Customer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 10, CusVer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 20, DayVer + 2, 10);
			XU_ReadFormSF(0x0148 + 11, FlashCodeVer, 5);
		}
		else if (RomValue[5] == 2){
			XU_ReadFormSF(0x0FD0, Customer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 18, CusVer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 28, DayVer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 10, FlashCodeVer, 5);
		}
		else{
			return false;
		}
		break;
	case 0x36:
		XU_ReadFormSF(0x3E00, Customer + 2, 10);
		XU_ReadFormSF(0x3E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x3E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x50:
		XU_ReadFormSF(0x5E00, Customer + 2, 10);
		XU_ReadFormSF(0x5E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x5E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x56:
		XU_ReadFormSF(0x3E00, Customer + 2, 10);
		XU_ReadFormSF(0x3E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x3E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x70:
		XU_ReadFormSF(0x3E05, Customer + 2, 10);
		XU_ReadFormSF(0x3E0F, CusVer + 2, 10);
		XU_ReadFormSF(0x3E19, DayVer + 2, 10);
		XU_ReadFormSF(0x3E00, FlashCodeVer, 5);
		break;
	case 0x71:
		XU_ReadFormSF(0x3E05, Customer + 2, 10);
		XU_ReadFormSF(0x3E0F, CusVer + 2, 10);
		XU_ReadFormSF(0x3E19, DayVer + 2, 10);
		XU_ReadFormSF(0x3E00, FlashCodeVer, 5);
		break;
	case 0x75:
		if (RomValue[5] == 1){

			if(romType == DRT_64K){
				XU_ReadFormSF(0x4E00, Customer + 2, 10);
				XU_ReadFormSF(0x4E12, CusVer + 2, 10);
				XU_ReadFormSF(0x4E1C, DayVer + 2, 10);
				XU_ReadFormSF(0x4E0A, FlashCodeVer, 5);
			}
			else if(romType == DRT_128K){
				XU_ReadFormSF(0x2000, Customer + 2, 10);
				XU_ReadFormSF(0x2012, CusVer + 2, 10);
				XU_ReadFormSF(0x201C, DayVer + 2, 10);
				XU_ReadFormSF(0x200A, FlashCodeVer, 5);
			}
			else
			{
				return false;
			}

		}
		else{
			return false;
		}

		break;
	case 0x76: //128K
		if (RomValue[5] == 1){
			XU_ReadFormSF(0x3FD0, Customer + 2, 10);
			XU_ReadFormSF(0x3FD0 + 10, CusVer + 2, 10);
			XU_ReadFormSF(0x3FD0 + 20, DayVer + 2, 10);
			XU_ReadFormSF(0x0148 + 11, FlashCodeVer, 5);
		}
		else{
			return false;
		}
		break;
	case 0x83: //128K
	case 0x85: //128K
	case 0x86: //128K
		XU_ReadFormSF(0x2000, Customer + 2, 10);
		XU_ReadFormSF(0x2012, CusVer + 2, 10);
		XU_ReadFormSF(0x201C, DayVer + 2, 10);
		XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		break;
	case 0x89:
	case 0x90:
		XU_ReadFormSF(0x2000, Customer + 2, 10);
		XU_ReadFormSF(0x2012, CusVer + 2, 10);
		XU_ReadFormSF(0x201D, DayVer + 2, 10);
		XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		break;
	case 0x92: //128K
		XU_ReadFormSF(0x2000, Customer + 2, 10);
		XU_ReadFormSF(0x2012, CusVer + 2, 10);
		XU_ReadFormSF(0x201C, DayVer + 2, 10);
		XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		break;
	default:
		return false;
		break;
	}

	//220
	if (chipID == 0x20){
		FlashCodeVer[40] = '\0';
	}
	else{
		FlashCodeVer[41] = '\0';
	}

if(bNormalExport){
		memcpy(FlashCodeVer + 5, CusVer + 2, 10);
		memcpy(FlashCodeVer + 15, Customer + 2, 10);
		memcpy(FlashCodeVer + 25, DayVer + 2, 10);
	}else{
		/* 第二种组合方式	*/
		BYTE cFLashCodeVer2 = FlashCodeVer[4];
		FlashCodeVer[4] = '-';
		int index = 0;
		BYTE *pFWVersion = FlashCodeVer;
		pFWVersion += 5;
		int i = 0;
		for (i = 0; i<10; i++)
		{
			if (CusVer[i + 2] != 0x2d)
			{
				*(pFWVersion++) = CusVer[i + 2];
			}
		}
		if ( 0 != memcmp(RomValue, "220R", 4))
		{
			*(pFWVersion++) = '-';
			*(pFWVersion++) = cFLashCodeVer2;
		}
		char cDayVer1[7] = {0};
		char cDayVer2[3] = {0};
		char cDayVer3[3] = {0};
		memcpy(cDayVer1, DayVer + 2, 6);
		memcpy(cDayVer2, DayVer + 8, 2);
		memcpy(cDayVer3, DayVer + 10, 2);

		*(pFWVersion++) = '-';
		*(pFWVersion++) = 'V';
		memcpy(pFWVersion, cDayVer1, 6);
		pFWVersion += 6;
		*(pFWVersion++) = '_';
		memcpy(pFWVersion, cDayVer2, 2);
		pFWVersion += 2;
		*(pFWVersion++) = '_';
		memcpy(pFWVersion, cDayVer3, 2);
		pFWVersion += 2;
		*(pFWVersion++) = '-';

		for (i = 0; i < 10; i++)
		{
			if (Customer[i + 2] != 0x2d)
			{
				*(pFWVersion++) = Customer[i + 2];
			}
		}
	}

	char Temp[56] = { 0 };
	memcpy(Temp, FlashCodeVer, 56);
	memset(pData, 0, len);
	memcpy(pData, Temp, (len > 56) ? 56 : len);

	return true;
}

bool  SonixCam_GetFwVersionFormFile(unsigned char pData[], long len, BOOL bNormalExport)
{
	if (!camera_init)
		return false;

	BYTE RomValue[10] = { 0 };
	BOOL hr = XU_GetAsicRomVersion(RomValue);

	BYTE chipID;
	DSP_ARCH_TYPE asicArchType;
	if (DAT_UNKNOW == XU_GetChipRomType(&chipID, &asicArchType))
		return false;
	
	BYTE FlashCodeVer[56] = { 0 };
	BYTE CusVer[31];
	BYTE DayVer[31];
	BYTE Customer[31];

	switch (chipID){
	case 0x15:
		XU_ReadFormSF(0x3E00, Customer + 2, 10);
		XU_ReadFormSF(0x3E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x3E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x16:
		XU_ReadFormSF(0x0FD0, Customer + 2, 10);
		XU_ReadFormSF(0x0FD0 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x0FD0 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x0148 + 11, FlashCodeVer, 5);
		break;
	case 0x20:
		XU_ReadFormSF(0x5E00, Customer + 2, 10);
		XU_ReadFormSF(0x5E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x5E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x0 + 11, FlashCodeVer, 4);
		break;
	case 0x25:
		XU_ReadFormSF(0x5E00, Customer + 2, 10);
		XU_ReadFormSF(0x5E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x5E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x30:
		XU_ReadFormSF(0x1E00, Customer + 2, 10);
		XU_ReadFormSF(0x1E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x1E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
	case 0x31:
		if (true != hr){
			return false;
		}
		if (RomValue[5] == 1){
			XU_ReadFormSF(0x1E00, Customer + 2, 10);
			XU_ReadFormSF(0x1E00 + 10, CusVer + 2, 10);
			XU_ReadFormSF(0x1E00 + 20, DayVer + 2, 10);
			XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		}
		else if (RomValue[5] == 2){
			XU_ReadFormSF(0x6000 + 0x1E0, Customer + 2, 10);
			XU_ReadFormSF(0x6000 + 0x1E00 + 10, CusVer + 2, 10);
			XU_ReadFormSF(0x6000 + 0x1E00 + 20, DayVer + 2, 10);
			XU_ReadFormSF(0x6000 + 0x01e8 + 11, FlashCodeVer, 5);
		}
		else{
			return false;
		}

		break;
	case 0x32:
		if (true != hr){
			return false;
		}
		if (RomValue[5] == 1){
			XU_ReadFormSF(0x0FD0, Customer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 10, CusVer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 20, DayVer + 2, 10);
			XU_ReadFormSF(0x0148 + 11, FlashCodeVer, 5);
		}
		else if (RomValue[5] == 2){
			XU_ReadFormSF(0x0FD0, Customer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 18, CusVer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 28, DayVer + 2, 10);
			XU_ReadFormSF(0x0FD0 + 10, FlashCodeVer, 5);
		}
		else{
			return false;
		}
		break;
	case 0x36:
		XU_ReadFormSF(0x3E00, Customer + 2, 10);
		XU_ReadFormSF(0x3E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x3E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x50:
		XU_ReadFormSF(0x5E00, Customer + 2, 10);
		XU_ReadFormSF(0x5E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x5E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x56:
		XU_ReadFormSF(0x3E00, Customer + 2, 10);
		XU_ReadFormSF(0x3E00 + 10, CusVer + 2, 10);
		XU_ReadFormSF(0x3E00 + 20, DayVer + 2, 10);
		XU_ReadFormSF(0x01e8 + 11, FlashCodeVer, 5);
		break;
	case 0x70:
		XU_ReadFormSF(0x3E05, Customer + 2, 10);
		XU_ReadFormSF(0x3E0F, CusVer + 2, 10);
		XU_ReadFormSF(0x3E19, DayVer + 2, 10);
		XU_ReadFormSF(0x3E00, FlashCodeVer, 5);
		break;
	case 0x71:
		XU_ReadFormSF(0x3E05, Customer + 2, 10);
		XU_ReadFormSF(0x3E0F, CusVer + 2, 10);
		XU_ReadFormSF(0x3E19, DayVer + 2, 10);
		XU_ReadFormSF(0x3E00, FlashCodeVer, 5);
		break;
	case 0x75:
		if (RomValue[5] == 1){
			XU_ReadFormSF(0x2000, Customer + 2, 10);
			XU_ReadFormSF(0x2012, CusVer + 2, 10);
			XU_ReadFormSF(0x201C, DayVer + 2, 10);
			XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		}
		else{
			return false;
		}

		break;
	case 0x76: //128K
		if (RomValue[5] == 1){
			XU_ReadFormSF(0x3FD0, Customer + 2, 10);
			XU_ReadFormSF(0x3FD0 + 10, CusVer + 2, 10);
			XU_ReadFormSF(0x3FD0 + 20, DayVer + 2, 10);
			XU_ReadFormSF(0x0148 + 11, FlashCodeVer, 5);
		}
		else{
			return false;
		}
		break;
	case 0x83: //128K
		XU_ReadFormSF(0x2000, Customer + 2, 10);
		XU_ReadFormSF(0x2012, CusVer + 2, 10);
		XU_ReadFormSF(0x201C, DayVer + 2, 10);
		XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		break;
	case 0x85: //128K
		XU_ReadFormSF(0x2000, Customer + 2, 10);
		XU_ReadFormSF(0x2012, CusVer + 2, 10);
		XU_ReadFormSF(0x201C, DayVer + 2, 10);
		XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		break;
	case 0x86: //128K
		XU_ReadFormSF(0x2000, Customer + 2, 10);
		XU_ReadFormSF(0x2012, CusVer + 2, 10);
		XU_ReadFormSF(0x201C, DayVer + 2, 10);
		XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		break;	
	case 0x90:
		XU_ReadFormSF(0x2000, Customer + 2, 10);
		XU_ReadFormSF(0x2012, CusVer + 2, 10);
		XU_ReadFormSF(0x201C, DayVer + 2, 10);
		XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		break;
	case 0x92: //128K
		XU_ReadFormSF(0x2000, Customer + 2, 10);
		XU_ReadFormSF(0x2012, CusVer + 2, 10);
		XU_ReadFormSF(0x201C, DayVer + 2, 10);
		XU_ReadFormSF(0x200A, FlashCodeVer, 5);
		break;
	default:
		return false;
		break;
	}

	//220
	if (chipID == 0x20){
		FlashCodeVer[40] = '\0';
	}
	else{
		FlashCodeVer[41] = '\0';
	}

if(bNormalExport){
		memcpy(FlashCodeVer + 5, CusVer + 2, 10);
		memcpy(FlashCodeVer + 15, Customer + 2, 10);
		memcpy(FlashCodeVer + 25, DayVer + 2, 10);
	}else{
		/* 第二种组合方式	*/
		BYTE cFLashCodeVer2 = FlashCodeVer[4];
		FlashCodeVer[4] = '-';
		int index = 0;
		BYTE *pFWVersion = FlashCodeVer;
		pFWVersion += 5;
		int i = 0;
		for (i = 0; i<10; i++)
		{
			if (CusVer[i + 2] != 0x2d)
			{
				*(pFWVersion++) = CusVer[i + 2];
			}
		}
		if ( 0 != memcmp(RomValue, "220R", 4))
		{
			*(pFWVersion++) = '-';
			*(pFWVersion++) = cFLashCodeVer2;
		}
		char cDayVer1[7] = {0};
		char cDayVer2[3] = {0};
		char cDayVer3[3] = {0};
		memcpy(cDayVer1, DayVer + 2, 6);
		memcpy(cDayVer2, DayVer + 8, 2);
		memcpy(cDayVer3, DayVer + 10, 2);

		*(pFWVersion++) = '-';
		*(pFWVersion++) = 'V';
		memcpy(pFWVersion, cDayVer1, 6);
		pFWVersion += 6;
		*(pFWVersion++) = '_';
		memcpy(pFWVersion, cDayVer2, 2);
		pFWVersion += 2;
		*(pFWVersion++) = '_';
		memcpy(pFWVersion, cDayVer3, 2);
		pFWVersion += 2;
		*(pFWVersion++) = '-';

		for (i = 0; i < 10; i++)
		{
			if (Customer[i + 2] != 0x2d)
			{
				*(pFWVersion++) = Customer[i + 2];
			}
		}
	}

	char Temp[56] = { 0 };
	memcpy(Temp, FlashCodeVer, 56);
	memset(pData, 0, len);
	memcpy(pData, Temp, (len > 56) ? 56 : len);

	return true;
}


bool SonixCam_XuRead(unsigned char pData[], unsigned int length,  BYTE unitID, BYTE cs)
{
	if (!camera_init)
		return false;

	if(!XU_Read(pData, length, unitID, cs))
		return false;
	return true;
}


bool SonixCam_XuWrite(unsigned char pData[], unsigned int length,  BYTE unitID, BYTE cs)
{
	if (!camera_init)
		return false;

	if(!XU_Write(pData, length, unitID, cs))
		return false;
	return true;
}

bool  SonixCam_SensorRegisterCustomRead(unsigned char slaveId, unsigned short  addr, unsigned short  addrByteNumber, unsigned char pData[], long dataByteNumber, bool pollSCL)
{
	if (!camera_init)
		return false;

	USHORT temp = 0;
	if (!XU_CustomReadFromSensor(slaveId, addr, addrByteNumber, &temp, dataByteNumber, pollSCL))
		return false;
	if(dataByteNumber > 1){
		*pData++ = (BYTE)(temp >> 8);
		*pData++ = (BYTE)temp;
	}else{
		*pData = (BYTE)temp;
	}

	return true;
}

bool  SonixCam_SensorRegisterCustomWrite(unsigned char slaveId, unsigned short addr, unsigned short  addrByteNumber, unsigned char pData[], long dataByteNumber, bool pollSCL)
{
	if (!camera_init)
		return false;

	USHORT temp = 0;
	if(dataByteNumber > 1){
		temp = (USHORT)((USHORT)pData[0] << 8) + pData[1];
	}else{
		temp = pData[0];
	}
	if (!XU_CustomWriteToSensor(slaveId, addr, addrByteNumber, temp, dataByteNumber, pollSCL))
		return false;

	return true;
}

bool  SonixCam_SensorTwoRegisterCustomRead(unsigned char slaveId, unsigned short  addr, unsigned short  addrByteNumber, unsigned char pData[], long dataByteNumber, bool pollSCL)
{
	if (!camera_init)
		return false;

	if (!XU_CustomReadFromSensorTwo(slaveId, addr, addrByteNumber, pData, dataByteNumber, pollSCL))
		return false;

	return true;
}

bool  SonixCam_SensorTwoRegisterCustomWrite(unsigned char slaveId, unsigned short addr, unsigned short  addrByteNumber, unsigned char pData[], long dataByteNumber, bool pollSCL)
{
	if (!camera_init)
		return false;

	USHORT temp = 0;
	if(dataByteNumber > 1){
		temp = (USHORT)((USHORT)pData[0] << 8) + pData[1];
	}else{
		temp = pData[0];
	}
	if (!XU_CustomWriteToSensorTwo(slaveId, addr, addrByteNumber, pData, dataByteNumber, pollSCL))
		return false;

	return true;
}


bool SonixCam_SerialFlashRead(long addr, unsigned char pData[], long len)
{
	if (!camera_init)
		return false;

	if(IsSupportXU6()){
		return ReadFormSF_XU6(addr, pData, len);
	}
	return XU_ReadFormSF(addr, pData, len);
}

bool SonixCam_SerialFlashSectorWrite(long addr, unsigned char pData[], long len, SERIAL_FLASH_TYPE sft)
{
	if (!camera_init)
		return false;

	if (sft == SFT_UNKNOW)
	{	
		return false;
	}

	//disable serial flash wirte protect
	if(!XU_DisableSerialFlashWriteProtect(sft))
		return false;

	//erase serial flash sector 
	if(!XU_EraseSectorForSerialFlash(addr, sft))
		return false;

	sleep(1);

	if (IsSupportXU6()) {
		return WriteToSF_XU6(addr, pData, len);
	}
	return XU_WriteToSF(addr, pData, len);
}

bool SonixCam_SerialFlashWrite(long addr, unsigned char pData[], long len)
{
	if (!camera_init)
		return false;

	if(IsSupportXU6()){
		return WriteToSF_XU6(addr, pData, len);
	}
	return XU_WriteToSF(addr, pData, len);
}

bool SonixCam_SerialFlashSectorCustomWrite(long addr, unsigned char pData[], long len, SERIAL_FLASH_TYPE sft)
{
	if (!camera_init)
		return false;

	if (sft == SFT_UNKNOW)
	{	
		return false;
	}

	if(!XU_DisableSerialFlashWriteProtect(sft))
		return false;

	if(!XU_EraseSectorForSerialFlash(addr, sft))
		return false;
	sleep(1);

	XU_SFWaitReady();

	XU_EnableAsicRegisterBit(sfModeAddr, 0);			// Flash Mode:En
	//WREN cycle
	XU_DisableAsicRegisterBit(sfCSAddr, 0);			// CS:0
	BYTE data = 6;
	XU_WriteToASIC(sfWriteDataAddr, data);					//WREN
	XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
	XU_SFWaitReady();
	XU_EnableAsicRegisterBit(sfCSAddr, 0);			// CS:1

	LONG startAddr = addr;
	if (sft == SFT_SST)
	{
		XU_DisableAsicRegisterBit(sfCSAddr, 0);
		data = 0xaf;
		XU_WriteToASIC(sfWriteDataAddr, data);
		XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
		XU_SFWaitReady();

		data = startAddr >> 16;
		XU_WriteToASIC(sfWriteDataAddr, data);
		XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
		XU_SFWaitReady();
		data = startAddr >> 8;
		XU_WriteToASIC(sfWriteDataAddr, data);				//addr2
		XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
		XU_SFWaitReady();
		data = (BYTE)startAddr;
		XU_WriteToASIC(sfWriteDataAddr, data);				//addr3
		XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
		XU_SFWaitReady();
		int i = 0;
		for (i = 0; i < len; i++)
		{
			XU_WriteToASIC(sfWriteDataAddr, pData[i]);		//Data
			XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
			XU_SFWaitReady();
			XU_EnableAsicRegisterBit(sfCSAddr, 0);				// CS:1
			if (i == len - 1)
				break;
			XU_DisableAsicRegisterBit(sfCSAddr, 0);				// CS:0
			data = 0xaf;
			XU_WriteToASIC(sfWriteDataAddr, data);				//PP  [0xaf:SST 0x2:other]
			XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
			XU_SFWaitReady();
		}
		//WRDI
		XU_DisableAsicRegisterBit(sfCSAddr, 0);			// CS:0
		data = 4;
		XU_WriteToASIC(sfWriteDataAddr, data);
		XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
		XU_SFWaitReady();
		XU_EnableAsicRegisterBit(sfCSAddr, 0);			// CS:1
		usleep(100000);
		XU_SFWaitReady();
	}
	else if (sft == SFT_MXIC || sft == SFT_GIGA || sft == SFT_WINBOND || sft == SFT_MXIC_LIKE)
	{
		LONG j = 0;
		do{
			//WREN cycle
			XU_DisableAsicRegisterBit(sfCSAddr, 0);			// CS:0
			BYTE data = 6;
			XU_WriteToASIC(sfWriteDataAddr, data);					//WREN
			XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
			XU_SFWaitReady();
			XU_EnableAsicRegisterBit(sfCSAddr, 0);			// CS:1

			XU_DisableAsicRegisterBit(sfCSAddr, 0);			// CS:0
			data = 2;
			XU_WriteToASIC(sfWriteDataAddr, data);
			XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
			XU_SFWaitReady();								//addr1
			data = startAddr >> 16;
			XU_WriteToASIC(sfWriteDataAddr, data);
			XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
			XU_SFWaitReady();
			data = startAddr >> 8;
			XU_WriteToASIC(sfWriteDataAddr, data);				//addr2
			XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
			XU_SFWaitReady();
			data = (BYTE)startAddr;
			XU_WriteToASIC(sfWriteDataAddr, data);				//addr3
			XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
			XU_SFWaitReady();
			for (; j < len;)
			{
				XU_WriteToASIC(sfWriteDataAddr, pData[j]);		//Data
				XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
				XU_SFWaitReady();
				startAddr++;
				j++;
				if (0 == startAddr % 256)
				{
					XU_EnableAsicRegisterBit(sfCSAddr, 0);	// CS:1
					break;
				}
			}
			if (j == len)
			{
				XU_EnableAsicRegisterBit(sfCSAddr, 0);	// CS:1
				break;
			}
		} while (1);
	}
	else
	{
		XU_DisableAsicRegisterBit(sfCSAddr, 0);				// CS:0
		data = 2;									//PP  modify from ldata = 2;  // 0x2:other   0xaf:SST
		XU_WriteToASIC(sfWriteDataAddr, data);
		XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
		XU_SFWaitReady();
		//ldata = 0;									//addr1
		data = startAddr >> 16;
		XU_WriteToASIC(sfWriteDataAddr, data);
		XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
		XU_SFWaitReady();
		data = startAddr >> 8;
		XU_WriteToASIC(sfWriteDataAddr, data);		//addr2
		XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
		XU_SFWaitReady();
		data = (BYTE)startAddr;
		XU_WriteToASIC(sfWriteDataAddr, data);		//addr3
		XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
		XU_SFWaitReady();
		int i = 0;
		for (i = 0; i < len; i++)
		{
			XU_WriteToASIC(sfWriteDataAddr, pData[i]);  //Data
			XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
			XU_SFWaitReady();
		}

		XU_EnableAsicRegisterBit(sfCSAddr, 0);				// CS:1
	}
	XU_DisableAsicRegisterBit(sfModeAddr, 0);				// Flash Mode:Dis

}

bool SonixCam_SerialFlashCustomRead(long addr, unsigned char pData[], long len)
{
	if (!camera_init)
		return false;

	XU_EnableAsicRegisterBit(sfModeAddr, 0);//series flash mode
	XU_DisableAsicRegisterBit(sfCSAddr, 0);//CS -> 0

	BYTE data = 3;
	XU_WriteToASIC(0x1088, data);//read command 0x1088 = 0x03
	XU_WriteToASIC(sfWriteDataAddr, data);
	XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);

	LONG startAddr = addr;
	data = startAddr >> 16;
	XU_WriteToASIC(0x1089, data);
	XU_WriteToASIC(sfWriteDataAddr, data);//addr0
	XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
	XU_SFWaitReady();

	data = startAddr >> 8;
	XU_WriteToASIC(0x108a, data);
	XU_WriteToASIC(sfWriteDataAddr, data);//addr1
	XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
	XU_SFWaitReady();

	data = (BYTE)startAddr;
	XU_WriteToASIC(0x108b, data);
	XU_WriteToASIC(sfWriteDataAddr, data);//addr2
	XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 0);
	XU_SFWaitReady();

	int i = 0;
	for (i = 0; i < len; i++)
	{
		data = 0;
		XU_WriteToASIC(sfReadDataAddr, data);//ready for read
		XU_EnableAsicRegisterBit(sfReadWriteTriggerAddr, 1);	//read TRG
		XU_SFWaitReady();//read 1084 bit0 when bit0 is 1 means ISP controler is ready
		//500us timeout
		XU_ReadFromASIC(sfReadDataAddr, &pData[i]);//read data
	}
	XU_EnableAsicRegisterBit(sfCSAddr, 0);//CS -> 1
	XU_DisableAsicRegisterBit(sfModeAddr, 0);//series flash mode disable	

	return true;
}

bool  SonixCam_GetManufacturer(unsigned char pData[], long len)
{
	if (!camera_init)
		return false;
	return XU_GetStringSettingFormSF(pData, len, 0x80, true);
}

bool  SonixCam_GetProduct(unsigned char pData[], long len)
{
	if (!camera_init)
		return false;

	return XU_GetStringSettingFormSF(pData, len, 0x40, true);
}

bool  SonixCam_GetVidPid(unsigned char pData[], long len)
{
	if (!camera_init)
		return false;

	DWORD dwStringAddr = 0;
	ULONG dwParaTableStartAddr = 0;
	ULONG dwParaTableEndAddr = 0;
	ULONG dwCRCStartAddr = 0;
	if (!XU_GetParaTableAndCRCAddrFormSF(&dwParaTableStartAddr, &dwParaTableEndAddr, &dwCRCStartAddr))
		return false;

	dwStringAddr = dwParaTableStartAddr + 0x06;
	BYTE pbyStringBuf[4] = { 0 };
	if (!XU_ReadFormSF(dwStringAddr, pbyStringBuf, sizeof(pbyStringBuf)))
		return false;

	memcpy(pData, pbyStringBuf, 4);
	return true;
}

bool SonixCam_GetString3(unsigned char pData[], long len)
{
	if (!camera_init)
		return false;

	return XU_GetStringSettingFormSF(pData, len, 0x100, true);
}

bool SonixCam_GetInterface(unsigned char pData[], long len)
{
	if (!camera_init)
		return false;

	return XU_GetStringSettingFormSF(pData, len, 0x140, true);
}

bool  SonixCam_GetSerialNumber(unsigned char pData[], long len)
{
	if (!camera_init)
		return false;

	return XU_GetStringSettingFormSF(pData, len, 0xC0, true);
}

bool SonixCam_DisableSerialFlashWriteProtect(SERIAL_FLASH_TYPE sft)
{
	if (!camera_init)
		return false;
	
	if (!XU_DisableSerialFlashWriteProtect(sft))
	{
		return false;
	}
	return true;
}

bool SonixCam_EraseSerialFlash(SERIAL_FLASH_TYPE sft)
{
	if (!camera_init)
		return false;

	if (!XU_DisableSerialFlashWriteProtect(sft))
	{
		return false;
	}

	if (!XU_SerialFlashErase(sft))
	{
		return false;
	}
}

bool SonixCam_EraseSectorFlash(long addr, SERIAL_FLASH_TYPE sft)
{
	if (!camera_init)
		return false;

	if (!XU_DisableSerialFlashWriteProtect(sft))
	{
		return false;
	}

	if (!XU_EraseSectorForSerialFlash(addr, sft))
	{
		return false;
	}
	return true;
}

bool SonixCam_EraseBlockFlash(long addr, SERIAL_FLASH_TYPE sft)
{
	if (!camera_init)
		return false;

	if (!XU_DisableSerialFlashWriteProtect(sft))
	{
		return false;
	}

	if (!XU_EraseBlockForSerialFlash(addr, sft))
	{
		return false;
	}
	return true;
}

bool SonixCam_GetAsicRomType(DSP_ROM_TYPE *romType, unsigned char *chipID)
{
	if (!camera_init)
		return false;

	BYTE id; 
	DSP_ARCH_TYPE dspArchType;
	*romType = XU_GetChipRomType(&id, &dspArchType);
	*chipID = id;
	return true;
}

bool SonixCam_BurnerFW(unsigned char pFwBuffer[], LONG lFwLength, SonixCam_SetProgress setProgress, void *ptrClass, SERIAL_FLASH_TYPE sft, BOOL bFullCheckFW)
{
	if (!camera_init)
		return false;

	if (sft == SFT_UNKNOW)
	{	
		return false;
	}
	
	BYTE chipID;
	DSP_ARCH_TYPE asicArchType;
	DSP_ROM_TYPE romType = XU_GetChipRomType(&chipID, &asicArchType);
	if (romType == DRT_Unknow)
		return false;
	switch (chipID)
	{
	case 0x85:
		XU_WriteToASIC(0x5FF, 0x5A);
		XU_WriteToASIC(0xA00, 0x1); // first frame for tw
		XU_WriteToASIC(0xA0A, 0x1); // first frame for sz
		break;
	case 0x86:
		XU_WriteToASIC(0xA00, 0x1); // first frame for tw
		XU_WriteToASIC(0xA0A, 0x1); // first frame for sz
		break;
	default:
		break;
	}
	if (true != XU_DisableSerialFlashWriteProtect(sft))
	{
		return false;
	}
	if (true != XU_SerialFlashErase(sft))
	{
		return false;
	}
	sleep(1);

	//erase check
	LONG i;
	LONG fwLen = lFwLength;
	BYTE data[8];
	BYTE temp[8];
	memset(temp, 0xFF, sizeof(temp));
	for (i = 0; i < fwLen - 1024; i += 1024){
		memset(&data, 0xff, 8);
		if (!XU_ReadDataFormFlash(i, data, 8))
			return false;
		if (0 != memcmp(data, temp, 8))
		{
			return false;
		}
	}

	/*********************************************************************************/
	/*********************************************************************************/
	/* 将0x160位置的四个字节设置为0xFF,在烧录固件过程中拔掉设备后，在插上设备
	dsp首先检查0x160是否是有效数据，如果是0xFF则固件无效，使用dsp rom内的固
	件启动设备，否则加载flash里面的固件启动设备。*/
	BYTE *pCopyFW = (BYTE*)malloc(lFwLength);
	fwLen = lFwLength;
	if (!pCopyFW)
	{
		return false;
	}
	memcpy(pCopyFW, pFwBuffer, lFwLength);
	BYTE intBuffer[8] = { 0 };
	memcpy(intBuffer, pCopyFW + 0x160, 8);
	memset(pCopyFW + 0x160, 0xFF, 4);

	//259断电保护
	BYTE intBuffer_259_P1[2] = { 0 };
	BYTE intBuffer_259_P2[2] = { 0 };
	BYTE intBuffer_259_P3[21] = { 0 };
	if (chipID == 0x16)
	{
		memcpy(intBuffer_259_P1, pCopyFW, 2);
		memcpy(intBuffer_259_P2, pCopyFW + 0x6700, 2);
		memcpy(intBuffer_259_P3, pCopyFW + 0x6702, 0x15);
		memset(pCopyFW, 0xFF, 2);
		memset(pCopyFW + 0x6700, 0xFF, 0x17);
	}
	/*********************************************************************************/
	/*********************************************************************************/

	typedef BOOL(*PTR_WriteDataToFlash)(LONG addr, BYTE pData[], BYTE dataLen);
	PTR_WriteDataToFlash ptrWriteDataToFlash = &XU_WriteDataToFlash;
	float gProgress = 0;
	LONG step = 8;
	if(IsSupportXU6()){
		step = DF_XU_DATA_SIZE_SF64;
		ptrWriteDataToFlash = &WriteDataToFlashXU6;
		SetSFControllerSCK();
		SetWriteInfo_SFXU6(0, fwLen, DF_XU_DATA_SIZE_SF64);
	}
	for (i = 0; i < fwLen; i += step){
		if (setProgress && ((i % 0x200) == 0))
		{
			gProgress = (float)i / (float)fwLen;
			if (bFullCheckFW)
				gProgress *= 0.5f;
			setProgress(ptrClass, gProgress);
		}
		memset(data, 0xff, step);
		memcpy(data, pCopyFW + i, step);
		if (!ptrWriteDataToFlash(i, data, step)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return FALSE;
		}
	}


	/*********************************************************************************/
	/*********************************************************************************/
	/*烧录完成，恢复0x160地址数据*/
	if (!XU_WriteDataToFlash(0x160, intBuffer, 8)){
		SAFE_DELETE_ARRAY(pCopyFW);
		return false;
	}
	memcpy(pCopyFW + 0x160, intBuffer, 8);

	//259断电保护
	if (chipID == 0x16)
	{
		if (!XU_WriteDataToFlash(0x6702, intBuffer_259_P3, 8)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}
		if (!XU_WriteDataToFlash(0x6702 + 8, intBuffer_259_P3 + 8, 8)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}
		if (!XU_WriteDataToFlash(0x6702 + 16, intBuffer_259_P3 + 16, 5)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}

		if (!XU_WriteDataToFlash(0, intBuffer_259_P1, 2)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}

		if (!XU_WriteDataToFlash(0x6700, intBuffer_259_P2, 2)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}

		memcpy(pCopyFW, intBuffer_259_P1, 2);
		memcpy(pCopyFW + 0x6700, intBuffer_259_P2, 2);
		memcpy(pCopyFW + 0x6702, intBuffer_259_P3, 0x15);
	}
	/*********************************************************************************/
	/*********************************************************************************/

	if (bFullCheckFW)
	{
		if (setProgress)
			setProgress(ptrClass, 0.5f);
	}
	else
	{
		if(setProgress)
			setProgress(ptrClass, 1.0f);
	}

	step = 1024;
	LONG checkStep = 8;
	typedef BOOL(*PTR_ReadDataFromFlash)(LONG addr, BYTE pData[], BYTE dataLen);
	PTR_ReadDataFromFlash ptrReadDataFromFlash = &XU_ReadDataFormFlash;
	if (bFullCheckFW){
		step = 8;
		if (IsSupportXU6()) {
			step = DF_XU_DATA_SIZE_SF64;
			checkStep = DF_XU_DATA_SIZE_SF64;
			ptrReadDataFromFlash = &ReadDataFormFlashXU6;
			SetSFControllerSCK();
			SetReadInfo_SFXU6(0, fwLen, DF_XU_DATA_SIZE_SF64);
		}
	}
	for (i = 0; i < fwLen; i += step){
		memset(&data, 0xff, checkStep);
		if (!ptrReadDataFromFlash(i, data, checkStep)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return FALSE;
		}
		if (0 != memcmp(data, pCopyFW + i, checkStep))
		{
			SAFE_DELETE_ARRAY(pCopyFW);
			return FALSE;
		}
		if (bFullCheckFW && setProgress && ((i % 0x200) == 0))
		{
			gProgress = ((float)i / (float)fwLen) * 0.5;
			gProgress += 0.5f;
			setProgress(ptrClass, gProgress);
		}
	}
	if (bFullCheckFW)
	{
		if (setProgress)
			setProgress(ptrClass, 1.0f);
	}
	SAFE_DELETE_ARRAY(pCopyFW);
	return true;
}

BOOL CheckCRC(BYTE *pFW, LONG paraTableStartAddr, LONG paraTableLength, LONG crcStartAddr)
{
	USHORT crc16 = 0xFFFF;
	BYTE temp;
	LONG i, j;
	BYTE *pParaBuffer = pFW + paraTableStartAddr;
	for (i = 0; i < paraTableLength; i++)
	{
		temp = pParaBuffer[i];
		crc16 ^= temp;
		for (j = 0; j < 8; j++)
		{
			if (crc16 & 0x01)
			{
				crc16 >>= 1;
				crc16 ^= 0xA001;
			}
			else
				crc16 >>= 1;
		}
	}
	*(pFW + crcStartAddr + 20) = crc16 >> 8;
	*(pFW + crcStartAddr + 21) = crc16;
	return true;
}

BOOL SetParamToParamBuffer(BYTE paramBuffer[], LONG paramAddr, BYTE param[], LONG length)
{
	BYTE *pParamTable = paramBuffer + paramAddr;
	memset(pParamTable, 0xFF, 64);
	pParamTable[0] = length * 2 + 2;
	pParamTable[1] = 0x03;
	LONG i = 0;
	for (i = 0; i < length; i++)
	{
		*(pParamTable + 2 + 2 * i) = param[i];
		*(pParamTable + 2 + 2 * i + 1) = 0x0;
	}
	return true;
}

bool SonixCam_CustomBurnerFW(const ChangeParamInfo paramInfo, unsigned char pFwBuffer[], LONG lFwLength, SonixCam_SetProgress setProgress, void *ptrClass, SERIAL_FLASH_TYPE sft, BOOL bFullCheckFW)
{
	if (!camera_init)
		return false;

	if (sft == SFT_UNKNOW)
	{
		return false;
	}

	DWORD dwStringAddr = 0;
	ULONG dwParaTableStartAddr = 0;
	ULONG dwParaTableEndAddr = 0;
	ULONG dwCRCStartAddr = 0;

	if (!XU_GetParaTableAndCRCAddrFormFW(pFwBuffer, &dwParaTableStartAddr, &dwParaTableEndAddr, &dwCRCStartAddr))
		return false;

	LONG fwLen = lFwLength;
	BYTE *pCopyFW = (BYTE*)malloc(lFwLength);
	fwLen = lFwLength;
	if (!pCopyFW)
	{
		return false;
	}
	memcpy(pCopyFW, pFwBuffer, lFwLength);

	BYTE data[8];
	BYTE temp[8];
	LONG addIndex = 0;
	float gProgress = 0.0;  //(0.0 - 1.0)

	if (paramInfo.pVidPid)
	{
		memcpy(pCopyFW + (dwParaTableStartAddr + 0x06), paramInfo.pVidPid, 4);
	}
	if (paramInfo.pProduct)
	{
		SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0x40), (BYTE*)paramInfo.pProduct, paramInfo.ProductLength);
	}
	if (paramInfo.pManufacture)
	{
		SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0x80), (BYTE*)paramInfo.pManufacture, paramInfo.ManufactureLength);
	}
	if (paramInfo.pSerialNumber)
	{
		SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0xC0), (BYTE*)paramInfo.pSerialNumber, paramInfo.SerialNumberLength);
	}
	if (paramInfo.pString3)
	{
		SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0x100), (BYTE*)paramInfo.pString3, paramInfo.String3Length);
	}
	if (paramInfo.pInterface)
	{
		SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0x140), (BYTE*)paramInfo.pInterface, paramInfo.InterfaceLength);
	}

	//CRC Check
	CheckCRC(pCopyFW, dwParaTableStartAddr, dwParaTableEndAddr - dwParaTableStartAddr, dwCRCStartAddr);


	//在285DSP的一些固件中，由于有上电直接开启图像，而开启图形无法烧录，所有要先
	//关闭图像，然后在烧录。
	BYTE chipID;
	DSP_ARCH_TYPE asicArchType;
	DSP_ROM_TYPE romType = XU_GetChipRomType(&chipID, &asicArchType);
	if (romType == DRT_Unknow){
		SAFE_DELETE_ARRAY(pCopyFW);
		return false;
	}
	switch (chipID)
	{
	case 0x85:
		XU_WriteToASIC(0x5FF, 0x5A);
		XU_WriteToASIC(0xA00, 0x1); // first frame for tw
		XU_WriteToASIC(0xA0A, 0x1); // first frame for sz
		break;
	case 0x86:
		XU_WriteToASIC(0xA00, 0x1); // first frame for tw
		XU_WriteToASIC(0xA0A, 0x1); // first frame for sz
		break;
	default:
		break;
	}

	//Diable flash wirte protect
	if (!XU_DisableSerialFlashWriteProtect(sft))
	{
		return false;
	}

	//erase flash
	if (!XU_SerialFlashErase(sft))
	{
		return false;
	}
	sleep(1);

	LONG i = 0;
	//erase check
	memset(temp, 0xFF, sizeof(temp));
	for (i = 0; i < fwLen - 1024; i += 1024){
		memset(&data, 0xff, 8);
		if (!XU_ReadDataFormFlash(i, data, 8))
			return false;
		if (0 != memcmp(data, temp, 8))
		{
			return false;
		}
	}

	/*********************************************************************************/
	/*********************************************************************************/
	/* 将0x160位置的四个字节设置为0xFF,在烧录固件过程中拔掉设备后，在插上设备
	dsp首先检查0x160是否是有效数据，如果是0xFF则固件无效，使用dsp rom内的固
	件启动设备，否则加载flash里面的固件启动设备。*/
	BYTE intBuffer[8] = { 0 };
	memcpy(intBuffer, pCopyFW + 0x160, 8);
	memset(pCopyFW + 0x160, 0xFF, 4);

	//259断电保护
	BYTE intBuffer_259_P1[2] = { 0 };
	BYTE intBuffer_259_P2[2] = { 0 };
	BYTE intBuffer_259_P3[21] = { 0 };
	if (chipID == 0x16)
	{
		memcpy(intBuffer_259_P1, pCopyFW, 2);
		memcpy(intBuffer_259_P2, pCopyFW + 0x6700, 2);
		memcpy(intBuffer_259_P3, pCopyFW + 0x6702, 0x15);
		memset(pCopyFW, 0xFF, 2);
		memset(pCopyFW + 0x6700, 0xFF, 0x17);
	}
	/*********************************************************************************/
	/*********************************************************************************/
	
	typedef BOOL(*PTR_WriteDataToFlash)(LONG addr, BYTE pData[], BYTE dataLen);
	PTR_WriteDataToFlash ptrWriteDataToFlash = &XU_WriteDataToFlash;
	LONG step = 8;
	if(IsSupportXU6()){
		step = DF_XU_DATA_SIZE_SF64;
		ptrWriteDataToFlash = &WriteDataToFlashXU6;
		SetSFControllerSCK();
		SetWriteInfo_SFXU6(0, fwLen, DF_XU_DATA_SIZE_SF64);
	}
	for (i = 0; i < fwLen; i += step){
		if (setProgress && ((i % 0x200) == 0))
		{
			gProgress = (float)i / (float)fwLen;
			if (bFullCheckFW)
				gProgress *= 0.5f;
			setProgress(ptrClass, gProgress);
		}
		memset(data, 0xff, step);
		memcpy(data, pCopyFW + i, step);
		if (!ptrWriteDataToFlash(i, data, step)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return FALSE;
		}
	}

	/*********************************************************************************/
	/*********************************************************************************/
	/*烧录完成，恢复0x160地址数据*/
	if (!XU_WriteDataToFlash(0x160, intBuffer, 8)){
		SAFE_DELETE_ARRAY(pCopyFW);
		return false;
	}
	memcpy(pCopyFW + 0x160, intBuffer, 8);

	//259断电保护
	if (chipID == 0x16)
	{
		if (!XU_WriteDataToFlash(0x6702, intBuffer_259_P3, 8)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}
		if (!XU_WriteDataToFlash(0x6702 + 8, intBuffer_259_P3 + 8, 8)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}
		if (!XU_WriteDataToFlash(0x6702 + 16, intBuffer_259_P3 + 16, 5)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}

		if (!XU_WriteDataToFlash(0, intBuffer_259_P1, 2)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}

		if (!XU_WriteDataToFlash(0x6700, intBuffer_259_P2, 2)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}

		memcpy(pCopyFW, intBuffer_259_P1, 2);
		memcpy(pCopyFW + 0x6700, intBuffer_259_P2, 2);
		memcpy(pCopyFW + 0x6702, intBuffer_259_P3, 0x15);
	}
	/*********************************************************************************/
	/*********************************************************************************/

	if (bFullCheckFW)
	{
		if (setProgress)
			setProgress(ptrClass, 0.5f);
	}
	else
	{
		if (setProgress)
			setProgress(ptrClass, 1.0f);
	}

	step = 1024;
	LONG checkStep = 8;
	typedef BOOL(*PTR_ReadDataFromFlash)(LONG addr, BYTE pData[], BYTE dataLen);
	PTR_ReadDataFromFlash ptrReadDataFromFlash = &XU_ReadDataFormFlash;
	if (bFullCheckFW){
		step = 8;
		if (IsSupportXU6()) {
			step = DF_XU_DATA_SIZE_SF64;
			checkStep = DF_XU_DATA_SIZE_SF64;
			ptrReadDataFromFlash = &ReadDataFormFlashXU6;
			SetSFControllerSCK();
			SetReadInfo_SFXU6(0, fwLen, DF_XU_DATA_SIZE_SF64);
		}
	}
	for (i = 0; i < fwLen; i += step){
		memset(&data, 0xff, checkStep);
		if (!ptrReadDataFromFlash(i, data, checkStep)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return FALSE;
		}
		if (0 != memcmp(data, pCopyFW + i, checkStep))
		{
			SAFE_DELETE_ARRAY(pCopyFW);
			return FALSE;
		}
		if (bFullCheckFW && setProgress && ((i % 0x200) == 0))
		{
			gProgress = ((float)i / (float)fwLen) * 0.5;
			gProgress += 0.5f;
			setProgress(ptrClass, gProgress);
		}
	}
	if (bFullCheckFW)
	{
		if (setProgress)
			setProgress(ptrClass, 1.0f);
	}
	SAFE_DELETE_ARRAY(pCopyFW);
}

bool  SonixCam_WriteFwToFlash(unsigned char pFwBuffer[], LONG lFwLength, SonixCam_SetProgress setProgress, void *ptrClass, BOOL bFullCheckFW)
{
	if (!camera_init)
		return false;

	BYTE chipID;
	DSP_ARCH_TYPE asicArchType;
	DSP_ROM_TYPE romType = XU_GetChipRomType(&chipID, &asicArchType);
	if (romType == DRT_Unknow)
		return false;
	switch (chipID)
	{
	case 0x85:
		XU_WriteToASIC(0x5FF, 0x5A);
		XU_WriteToASIC(0xA00, 0x1); // first frame for tw
		XU_WriteToASIC(0xA0A, 0x1); // first frame for sz
		break;
	case 0x86:
		XU_WriteToASIC(0xA00, 0x1); // first frame for tw
		XU_WriteToASIC(0xA0A, 0x1); // first frame for sz
		break;
	default:
		break;
	}

	/*********************************************************************************/
	/*********************************************************************************/
	/* 将0x160位置的四个字节设置为0xFF,在烧录固件过程中拔掉设备后，在插上设备
	dsp首先检查0x160是否是有效数据，如果是0xFF则固件无效，使用dsp rom内的固
	件启动设备，否则加载flash里面的固件启动设备。*/
	BYTE *pCopyFW = (BYTE*)malloc(lFwLength);
	LONG fwLen = lFwLength;
	if (!pCopyFW)
	{
		return false;
	}
	memcpy(pCopyFW, pFwBuffer, lFwLength);
	BYTE intBuffer[8] = { 0 };
	memcpy(intBuffer, pCopyFW + 0x160, 8);
	memset(pCopyFW + 0x160, 0xFF, 4);

	//259断电保护
	BYTE intBuffer_259_P1[2] = { 0 };
	BYTE intBuffer_259_P2[2] = { 0 };
	BYTE intBuffer_259_P3[21] = { 0 };
	if (chipID == 0x16)
	{
		memcpy(intBuffer_259_P1, pCopyFW, 2);
		memcpy(intBuffer_259_P2, pCopyFW + 0x6700, 2);
		memcpy(intBuffer_259_P3, pCopyFW + 0x6702, 0x15);
		memset(pCopyFW, 0xFF, 2);
		memset(pCopyFW + 0x6700, 0xFF, 0x17);
	}
	/*********************************************************************************/
	/*********************************************************************************/

	float gProgress = 0;
	int i = 0;
	BYTE data[DF_XU_DATA_SIZE_SF64];
	typedef BOOL(*PTR_WriteDataToFlash)(LONG addr, BYTE pData[], BYTE dataLen);
	PTR_WriteDataToFlash ptrWriteDataToFlash = &XU_WriteDataToFlash;
	LONG step = 8;
	if(IsSupportXU6()){
		step = DF_XU_DATA_SIZE_SF64;
		ptrWriteDataToFlash = &WriteDataToFlashXU6;
		SetSFControllerSCK();
		SetWriteInfo_SFXU6(0, fwLen, DF_XU_DATA_SIZE_SF64);
	}
	for (i = 0; i < fwLen; i += step){
		if (setProgress && ((i % 0x200) == 0))
		{
			gProgress = (float)i / (float)fwLen;
			if (bFullCheckFW)
				gProgress *= 0.5f;
			setProgress(ptrClass, gProgress);
		}
		memset(data, 0xff, step);
		memcpy(data, pCopyFW + i, step);
		if (!ptrWriteDataToFlash(i, data, step)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return FALSE;
		}
	}
	
	/*********************************************************************************/
	/*********************************************************************************/
	/*烧录完成，恢复0x160地址数据*/
	if (!XU_WriteDataToFlash(0x160, intBuffer, 8)){
		SAFE_DELETE_ARRAY(pCopyFW);
		return false;
	}
	memcpy(pCopyFW + 0x160, intBuffer, 8);

	//259断电保护
	if (chipID == 0x16)
	{
		if (!XU_WriteDataToFlash(0x6702, intBuffer_259_P3, 8)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}
		if (!XU_WriteDataToFlash(0x6702 + 8, intBuffer_259_P3 + 8, 8)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}
		if (!XU_WriteDataToFlash(0x6702 + 16, intBuffer_259_P3 + 16, 5)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}

		if (!XU_WriteDataToFlash(0, intBuffer_259_P1, 2)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}

		if (!XU_WriteDataToFlash(0x6700, intBuffer_259_P2, 2)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}

		memcpy(pCopyFW, intBuffer_259_P1, 2);
		memcpy(pCopyFW + 0x6700, intBuffer_259_P2, 2);
		memcpy(pCopyFW + 0x6702, intBuffer_259_P3, 0x15);
	}
	/*********************************************************************************/
	/*********************************************************************************/

	if (bFullCheckFW)
	{
		if (setProgress)
			setProgress(ptrClass, 0.5f);
	}
	else
	{
		if(setProgress)
			setProgress(ptrClass, 1.0f);
	}

	step = 1024;
	LONG checkStep = 8;
	typedef BOOL(*PTR_ReadDataFromFlash)(LONG addr, BYTE pData[], BYTE dataLen);
	PTR_ReadDataFromFlash ptrReadDataFromFlash = &XU_ReadDataFormFlash;
	if (bFullCheckFW){
		step = 8;
		if (IsSupportXU6()) {
			step = DF_XU_DATA_SIZE_SF64;
			checkStep = DF_XU_DATA_SIZE_SF64;
			ptrReadDataFromFlash = &ReadDataFormFlashXU6;
			SetSFControllerSCK();
			SetReadInfo_SFXU6(0, fwLen, DF_XU_DATA_SIZE_SF64);
		}
	}
	for (i = 0; i < fwLen; i += step){
		memset(&data, 0xff, checkStep);
		if (!ptrReadDataFromFlash(i, data, checkStep)){
			SAFE_DELETE_ARRAY(pCopyFW);
			return FALSE;
		}
		if (0 != memcmp(data, pCopyFW + i, checkStep))
		{
			SAFE_DELETE_ARRAY(pCopyFW);
			return FALSE;
		}
		if (bFullCheckFW && setProgress && ((i % 0x200) == 0))
		{
			gProgress = ((float)i / (float)fwLen) * 0.5;
			gProgress += 0.5f;
			setProgress(ptrClass, gProgress);
		}
	}
	if (bFullCheckFW)
	{
		if (setProgress)
			setProgress(ptrClass, 1.0f);
	}
	SAFE_DELETE_ARRAY(pCopyFW);
	return true;
}

bool SonixCam_ExportFW(unsigned char pFwBuffer[], LONG lFwLength, SonixCam_SetProgress setProgress, void *ptrClass)
{
	if (!camera_init)
		return false;

	BYTE temp[DF_XU_DATA_SIZE_SF64];
	BYTE* pFw = pFwBuffer;
	LONG fwLen = lFwLength;
	float gProgress = 0.0;  //(0.0 - 1.0)
	LONG i;
	typedef BOOL(*PTR_ReadDataFromFlash)(LONG addr, BYTE pData[], BYTE dataLen);
	PTR_ReadDataFromFlash ptrReadDataFromFlash = &XU_ReadDataFormFlash;
	LONG step = 8;
	if (IsSupportXU6()) {
		step = DF_XU_DATA_SIZE_SF64;
		ptrReadDataFromFlash = &ReadDataFormFlashXU6;
		SetSFControllerSCK();
		SetReadInfo_SFXU6(0, fwLen, DF_XU_DATA_SIZE_SF64);
	}
	for ( i= 0; i < fwLen; i += step){
		if (setProgress)
		{
			gProgress = (float)i / (float)fwLen;
			setProgress(ptrClass, gProgress);
		}
		memset(&temp, 0xff, step);
		if (!ptrReadDataFromFlash(i, temp, 8))
			return FALSE;
		memcpy(pFw + i, temp, step);
	}
	return true;
}

bool SonixCam_SetParamTableFromFWFile(unsigned char pFW[], long lFwLength, const ChangeParamInfo paramInfo, SonixCam_SetProgress setProgress, void *ptrClass, SERIAL_FLASH_TYPE sft, char* pLogFilePath, BOOL bFullCheckFW)
{
	if (!camera_init)
		return false;

	if (sft == SFT_UNKNOW)
	{
		return false;
	}

	FILE *pLogfd = NULL;
	if (pLogFilePath)
	{
		if (0 == (pLogfd = fopen(pLogFilePath, "a")))
		{
			printf("Can't open the Log.txt file\n");
		}
	}

	DWORD dwStringAddr = 0;
	ULONG dwParaTableStartAddr = 0;
	ULONG dwParaTableEndAddr = 0;
	ULONG dwCRCStartAddr = 0;
	if (!XU_GetParaTableAndCRCAddrFormFW(pFW, &dwParaTableStartAddr, &dwParaTableEndAddr, &dwCRCStartAddr))
		return false;

	LONG SZ_4K = 4 * 1024;
	LONG SZ_16K = 16 * 1024; 
	LONG SEA_4K = dwParaTableStartAddr / SZ_4K * SZ_4K;

	LONG startSectorEraseAddr = SEA_4K;

	BYTE *pCopyFW = (BYTE*)malloc(lFwLength);

	if (!pCopyFW)
		return false;
	memcpy(pCopyFW, pFW, lFwLength);

	LONG startAddr = startSectorEraseAddr;
	BYTE temp[8];
	BYTE addrLow;
	BYTE addrHigh;
	LONG addIndex = 0;
	float gProgress = 0.0;  //(0.0 - 1.0)

	if (pLogfd){
		fwrite("1.Set param table\n", 1, sizeof("1.Set param table\n"), pLogfd);
		fflush(pLogfd);
	}
	if (paramInfo.pVidPid)
	{
		BYTE *pBuffer = pCopyFW + (dwParaTableStartAddr + 0x06);
		memcpy(pBuffer, paramInfo.pVidPid, 4);
	}
	if (paramInfo.pProduct)
	{
		SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0x40), (BYTE*)paramInfo.pProduct, paramInfo.ProductLength);
	}
	if (paramInfo.pSerialNumber)
	{
		SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0xC0), (BYTE*)paramInfo.pSerialNumber, paramInfo.SerialNumberLength);
	}
	if (paramInfo.pManufacture)
	{
		SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0x80), (BYTE*)paramInfo.pManufacture, paramInfo.ManufactureLength);
	}
	if (paramInfo.pString3)
	{
		SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0x100), (BYTE*)paramInfo.pString3, paramInfo.String3Length);
	}
	if (paramInfo.pInterface)
	{
		SetParamToParamBuffer(pCopyFW, (dwParaTableStartAddr + 0x140), (BYTE*)paramInfo.pInterface, paramInfo.InterfaceLength);
	}

	if (pLogfd){
		fwrite("2.Start CRC Check\n", 1, sizeof("2.Start CRC Check\n"), pLogfd);
		fflush(pLogfd);
	}
	//CRC Check
	CheckCRC(pCopyFW, dwParaTableStartAddr, dwParaTableEndAddr - dwParaTableStartAddr, dwCRCStartAddr);

	if (pLogfd){
		fwrite("3.Start disable flash write protect\n", 1, sizeof("3.Start disable flash write protect\n"), pLogfd);
		fflush(pLogfd);
	}

	//disable flash write protect
	if (!XU_DisableSerialFlashWriteProtect(sft))
	{
		if (pLogfd){
			fwrite("Disable flash write prote fail\n", 1, sizeof("Disable flash write prote fail\n"), pLogfd);
			fflush(pLogfd);
			fclose(pLogfd);
		}
		SAFE_DELETE_ARRAY(pCopyFW);
		return false;
	}

	if (pLogfd){
		fwrite("4.Start erase flash\n", 1, sizeof("4.Start erase flash\n"), pLogfd);
		fflush(pLogfd);
	}
	//sector erase
	XU_EraseSectorForSerialFlash(dwParaTableStartAddr, (SERIAL_FLASH_TYPE)sft);
	sleep(1);

	BOOL bNeedEraseCrcSec = false;
	LONG crcStartEraseAddr = dwCRCStartAddr / SZ_4K * SZ_4K;
	if (crcStartEraseAddr < SEA_4K || crcStartEraseAddr > SEA_4K) //need erase crc sector
	{
		bNeedEraseCrcSec = true;
		XU_EraseSectorForSerialFlash(dwCRCStartAddr / SZ_4K * SZ_4K, (SERIAL_FLASH_TYPE)sft);
		sleep(1);
	}

	//erase check and get flash sector size
	BYTE temp1[8];
	memset(&temp1, 0xff, 8);
	LONG i = 0;
	for (i = 0; i < SZ_4K; i += 0x50)
	{
		if (bNeedEraseCrcSec)
		{
			memset(&temp, 0xff, 8);
			if (!XU_ReadDataFormFlash(crcStartEraseAddr + i, temp, 8)){
				SAFE_DELETE_ARRAY(pCopyFW);
				if (pLogfd){
					fwrite("Erase flash fail!\n", 1, sizeof("Erase flash fail!\n"), pLogfd);
					fflush(pLogfd);
					fclose(pLogfd);
				}
				return false;
			}
			if (0 != memcmp(temp, temp1, 8))
			{
				SAFE_DELETE_ARRAY(pCopyFW);
				if (pLogfd){
					fwrite("Erase flash fail!\n", 1, sizeof("Erase flash fail!\n"), pLogfd);
					fflush(pLogfd);
					fclose(pLogfd);
				}
				return false;
			}
		}
		memset(&temp, 0xff, 8);
		if (!XU_ReadDataFormFlash(startSectorEraseAddr + i, temp, 8)){
			SAFE_DELETE_ARRAY(pCopyFW);
			if (pLogfd){
				fwrite("Erase flash fail!\n", 1, sizeof("Erase flash fail!\n"), pLogfd);
				fflush(pLogfd);
				fclose(pLogfd);
			}
			return false;
		}
		if (0 != memcmp(temp, temp1, 8))
		{
			SAFE_DELETE_ARRAY(pCopyFW);
			if (pLogfd){
				fwrite("Erase flash fail!\n", 1, sizeof("Erase flash fail!\n"), pLogfd);
				fflush(pLogfd);
				fclose(pLogfd);
			}
			return false;
		}
	}
	if (pLogfd){
		fwrite("Erase flash success\n", 1, sizeof("Erase flash success\n"), pLogfd);
		fflush(pLogfd);
	}

	if (pLogfd){
		fwrite("6.Start burn param table data\n", 1, sizeof("6.Start burn param table data\n"), pLogfd);
		fflush(pLogfd);
	}

	//burner sector src date
	for (i = 0; i < SZ_4K; i += 8)
	{
		if (setProgress && i % 0x20)
		{
			gProgress = (float)i / (float)SZ_4K;
			if (bFullCheckFW)
			{
				gProgress *= 0.5f;
				setProgress(ptrClass, gProgress);
			}
		}

		if (bNeedEraseCrcSec)
		{
			memcpy(temp, pCopyFW + crcStartEraseAddr + i, 8);
			if (!XU_WriteDataToFlash(crcStartEraseAddr + i, temp, 8)){
				SAFE_DELETE_ARRAY(pCopyFW);
				if (pLogfd){
					fwrite("Burn param table data fail\n", 1, sizeof("Burn param table data fail\n"), pLogfd);
					fflush(pLogfd);
					fclose(pLogfd);
				}
				return false;
			}
		}
		memcpy(temp, pCopyFW + startSectorEraseAddr + i, 8);
		if (!XU_WriteDataToFlash(startSectorEraseAddr + i, temp, 8)){
			SAFE_DELETE_ARRAY(pCopyFW);
			if (pLogfd){
				fwrite("Burn param table data fail\n", 1, sizeof("Burn param table data fail\n"), pLogfd);
				fflush(pLogfd);
				fclose(pLogfd);
			}
			return false;
		}
	}
	if (bFullCheckFW)
	{
		gProgress *= 0.5f;
		if (setProgress)
			setProgress(ptrClass, gProgress);
	}
	else
	{
		gProgress *= 1.0f;
		if (setProgress)
			setProgress(ptrClass, gProgress);
	}
	if (pLogfd){
		fwrite("Burn param table data success\n", 1, sizeof("Burn param table data success\n"), pLogfd);
		fflush(pLogfd);
		fclose(pLogfd);
	}

	if (pLogfd){
		fwrite("7.Start check param table data\n", 1, sizeof("7.Start check param table data\n"), pLogfd);
		fclose(pLogfd);
	}
	//burn check
	LONG setp = 0x200;
	if (bFullCheckFW)
		setp = 8;
	for (i = 0; i < SZ_4K; i += setp)
	{
		if (setProgress && i % 0x20)
		{
			gProgress = (float)i / (float)SZ_4K;
			if (bFullCheckFW)
			{
				gProgress *= 0.5f + 0.5;
				setProgress(ptrClass, gProgress);
			}
		}

		if (bNeedEraseCrcSec)
		{
			memset(&temp, 0xFF, 8);
			if (!XU_ReadDataFormFlash(crcStartEraseAddr + i, temp, 8)){
				SAFE_DELETE_ARRAY(pCopyFW);
				if (pLogfd){
					fwrite("Check param table data fail\n", 1, sizeof("Check param table data fail\n"), pLogfd);
					fclose(pLogfd);
				}
				return false;
			}
			if (0 != memcmp(temp, pCopyFW + crcStartEraseAddr + i, 8))
			{
				if (pLogfd){
					fwrite("Check param table data fail\n", 1, sizeof("Check param table data fail\n"), pLogfd);
					fclose(pLogfd);
				}
				SAFE_DELETE_ARRAY(pCopyFW);
				return false;
			}
		}
		memset(&temp, 0xFF, 8);
		if (!XU_ReadDataFormFlash(startSectorEraseAddr + i, temp, 8)){
			SAFE_DELETE_ARRAY(pCopyFW);
			if (pLogfd){
				fwrite("Check param table data fail\n", 1, sizeof("Check param table data fail\n"), pLogfd);
				fclose(pLogfd);
			}
			return false;
		}
		if (0 != memcmp(temp, pCopyFW + startSectorEraseAddr + i, 8))
		{
			if (pLogfd){
				fwrite("Check param table data fail\n", 1, sizeof("Check param table data fail\n"), pLogfd);
				fclose(pLogfd);
			}
			SAFE_DELETE_ARRAY(pCopyFW);
			return false;
		}
	}
	if (bFullCheckFW)
	{
		gProgress *= 1.0f;
		if (setProgress)
			setProgress(ptrClass, gProgress);
	}
	if (pLogfd){
		fwrite("Check param table data success\n", 1, sizeof("Check param table data success\n"), pLogfd);
		fclose(pLogfd);
	}
	SAFE_DELETE_ARRAY(pCopyFW);
	return true;
}
