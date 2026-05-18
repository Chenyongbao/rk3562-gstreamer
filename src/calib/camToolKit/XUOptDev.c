
#include "XUOptDev.h"
#include "util.h"
#include <stdio.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include "SFData.h"
#include <string.h>


int fd = 0;
extern BOOL camera_init;
BYTE g_curExtendUnitID = 0x03;
BOOL g_supportI2C64 = false;
BOOL g_supportXU6 = false;
BYTE g_chipID = 0x0;

BOOL XU_OpenCamera(char *devPath)
{
	struct v4l2_capability cap;
	if((fd = open(devPath,O_RDWR | O_NONBLOCK)) < 0)
	{
		return FALSE;
	} 

	memset(&cap, 0, sizeof(cap));
	if(ioctl(fd,VIDIOC_QUERYCAP,&cap)<0)
	{
		printf("Error opening device %s : unable to query device.\n", devPath);
		close(fd);
		return FALSE;
	}
	printf("Device %s opened: %s.\n",devPath, cap.card);

	USHORT ExtendUnitID = 0;
	if(!(ExtendUnitID = XU_GetUVCExtendUnitID()))
		return FALSE;
	g_curExtendUnitID = ExtendUnitID;

	BYTE chipID;
	DSP_ARCH_TYPE dspArchType;
	DSP_ROM_TYPE romType = XU_GetChipRomType(&chipID, &dspArchType);
	if(DRT_Unknow == romType){
		return FALSE;
	}

	if(!XU_SetAsicArchInfo(dspArchType)){
		return FALSE;
	}
	g_chipID = chipID;

	return TRUE;
}

BOOL XU_CloseCamera()
{
	if(!camera_init)
		return FALSE;
	
	if(-1 == close(fd))
	{
		perror("Fail to close fd");	
		return FALSE;
	}
	g_supportI2C64 = FALSE;
	g_supportXU6 = FALSE;
	g_chipID = 0x0;
	return TRUE;
}

int XU_Get_Cur(__u8 xu_unit, __u8 xu_selector, __u16 xu_size, __u8 *xu_data)
{
	int err=0;
#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 36)
	struct uvc_xu_control_query xctrl;
	xctrl.unit = xu_unit;
	xctrl.selector = xu_selector;
	xctrl.query = UVC_GET_CUR;
	xctrl.size = xu_size;
	xctrl.data = xu_data;
	err=ioctl(fd, UVCIOC_CTRL_QUERY, &xctrl);
#else
	struct uvc_xu_control xctrl;	
	xctrl.unit = xu_unit;
	xctrl.selector = xu_selector;
	xctrl.size = xu_size;
	xctrl.data = xu_data;
	err=ioctl(fd, UVCIOC_CTRL_GET, &xctrl);
#endif	
	return err;
}

int XU_Set_Cur(__u8 xu_unit, __u8 xu_selector, __u16 xu_size, __u8 *xu_data)
{
	int err=0;
#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 36)
	struct uvc_xu_control_query xctrl;
	xctrl.unit = xu_unit;
	xctrl.selector = xu_selector;
	xctrl.query = UVC_SET_CUR;
	xctrl.size = xu_size;
	xctrl.data = xu_data;
	err=ioctl(fd, UVCIOC_CTRL_QUERY, &xctrl);
#else
	struct uvc_xu_control xctrl;	
	xctrl.unit = xu_unit;
	xctrl.selector = xu_selector;
	xctrl.size = xu_size;
	xctrl.data = xu_data;
	err=ioctl(fd, UVCIOC_CTRL_SET, &xctrl);
#endif		
	return err;
}

BOOL XU_RestartDevice()
{
	BYTE romVersion[10] = { 0 };
	if (TRUE != XU_GetAsicRomVersion(romVersion))
	{
		return FALSE;
	}
	if ((uiRomID == ROM220) || (uiRomID == ROM225)){
		return FALSE;
	}

	BYTE TempVar = 0;
	if(XU_ReadFromASIC(usbResetAddr, &TempVar)){
		TempVar &= 0xFE;
		XU_WriteToASIC(usbResetAddr, TempVar);		//Always Error because of HW reset, Ignore it.
	}
	return TRUE;
}

BOOL XU_ReadFromASIC(USHORT addr, BYTE *pValue)
{
	int ret = 0;
	__u8 ctrldata[4];

	//uvc_xu_control parmeters
	__u8 xu_unit= g_curExtendUnitID; 
	__u8 xu_selector= XU_SONIX_SYS_ASIC_RW;
	__u16 xu_size= 4;
	__u8 *xu_data= ctrldata;

	xu_data[0] = (addr & 0xFF);
	xu_data[1] = ((addr >> 8) & 0xFF);
	xu_data[2] = 0x0;
	xu_data[3] = 0xFF;		/* Dummy Write */
	
	/* Dummy Write */
	if ((ret=XU_Set_Cur(xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		printf("ioctl(UVCIOC_CTRL_SET) FAILED (%i) \n",ret);
		//if(ret==EINVAL)			printf("Invalid arguments\n");		
		return FALSE;
	}
	
	/* Asic Read */
	xu_data[3] = 0x00;
	if ((ret=XU_Get_Cur(xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		printf("ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",ret);
		//if(ret==EINVAL)			printf("Invalid arguments\n");
		return FALSE;
	}
	*pValue = xu_data[2];
	if(ret < 0)
		return FALSE;
	return TRUE;
}

BOOL XU_WriteToASIC(USHORT addr, BYTE value)
{
	int ret = 0;
	__u8 ctrldata[4];

	//uvc_xu_control parmeters
	__u8 xu_unit= g_curExtendUnitID; 
	__u8 xu_selector= XU_SONIX_SYS_ASIC_RW;
	__u16 xu_size= 4;
	__u8 *xu_data= ctrldata;

	xu_data[0] = (addr & 0xFF);			/* Addr Low */
	xu_data[1] = ((addr >> 8) & 0xFF);	/* Addr High */
	xu_data[2] = value;
	xu_data[3] = 0x0;					/* Normal Write */
	
	/* Normal Write */
	if ((ret=XU_Set_Cur(xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		printf("ioctl(UVCIOC_CTRL_SET) FAILED (%i) \n",ret);
		//if(ret==EINVAL)			printf("Invalid arguments\n");	
		return FALSE;	
	}
	
	if(ret < 0)
		return FALSE;
	return TRUE;
}


BOOL XU_SetAsicArchInfo(DSP_ARCH_TYPE dspArchType)
{
	if (dspArchType <= DAT_UNKNOW || dspArchType >= DSP_ARCH_COUNT)
		return FALSE;

	DSP_ROM_TYPE art = DRT_Unknow;
	BYTE chipID;
	DSP_ARCH_TYPE asicArchType = DAT_UNKNOW;
	if (DAT_UNKNOW != (art == XU_GetChipRomType(&chipID, &asicArchType))){
		if (chipID == 0x88 || chipID == 0x86 || chipID == 0x89 || chipID == 0x90){
			g_supportI2C64 = true;
		}
	}

	BYTE romVersion[10] = { 0 };
	if (XU_GetAsicRomVersion(romVersion)) {
		if (uiRomID== ROM276V1 || uiRomID == ROM288V1 || uiRomID == ROM289 || uiRomID == ROM290 || uiRomID == ROM286) {
			g_supportXU6 = true;
		}
	}

	dspIdAddr = g_AsicArchInfo[dspArchType].asicIdAddr;

	usbResetAddr = g_AsicArchInfo[dspArchType].usbResetAddr;

	gpioInputAddr = g_AsicArchInfo[dspArchType].gpioInputAddr;
	gpioOutputAddr = g_AsicArchInfo[dspArchType].gpioOutputAddr;
	gpioOEAddr = g_AsicArchInfo[dspArchType].gpioOEAddr;

	sfRdyAddr = g_AsicArchInfo[dspArchType].sfRdyAddr;
	sfModeAddr = g_AsicArchInfo[dspArchType].sfModeAddr;
	sfCSAddr = g_AsicArchInfo[dspArchType].sfCSAddr;
	sfWriteDataAddr = g_AsicArchInfo[dspArchType].sfWriteDataAddr;
	sfReadDataAddr = g_AsicArchInfo[dspArchType].sfReadDataAddr;
	sfReadWriteTriggerAddr = g_AsicArchInfo[dspArchType].sfReadWriteTriggerAddr;
	sfReadCmdAddr = g_AsicArchInfo[dspArchType].sfReadCmdAddr;
	sfReadDataStartAddr = g_AsicArchInfo[dspArchType].sfReadDataStartAddr;

	i2cDev = g_AsicArchInfo[dspArchType].i2cDev;
	i2cMode = g_AsicArchInfo[dspArchType].i2cMode;
	i2cTrg = g_AsicArchInfo[dspArchType].i2cTrg;
	i2cSclSelOD = g_AsicArchInfo[dspArchType].i2cSclSelOD;
	i2cSlaveID = g_AsicArchInfo[dspArchType].i2cSlaveID;
	i2cDataArrStartAddr = g_AsicArchInfo[dspArchType].i2cDataArrStartAddr;

	cpuRateAddr = g_AsicArchInfo[dspArchType].cpuRateAddr;
	return TRUE;
}


BOOL XU_GetChipID(LONG idAddr, BYTE *pChipID)
{
	BYTE id = 0;
	BOOL hr = TRUE;
	int i;
	for (i = 0; i < 3; i++)
	{
		hr = XU_ReadFromASIC(idAddr, &id);
		if (TRUE == hr)
			break;
	}

	if (TRUE != hr)
		return FALSE;

	*pChipID = id;
	return TRUE;
}

DSP_ROM_TYPE XU_GetChipRomType(BYTE *pChipID, DSP_ARCH_TYPE *pAsicArchType)
{
	DSP_ROM_TYPE ret = DRT_Unknow;
	BYTE dspID = 0;
	DSP_ARCH_TYPE dspArchType = DAT_UNKNOW;
	BYTE idIndex;
	for (idIndex = 0; idIndex < DSP_ARCH_COUNT; idIndex++)
	{
		if (!XU_GetChipID(g_AsicArchInfo[idIndex].asicIdAddr, &dspID))
			return ret;

		switch (dspID)
		{
		case 0x15:
		case 0x16:
		case 0x22:
		case 0x23:
		case 0x25:
		case 0x32:
		case 0x33:
		case 0x56:
		case 0x70:
		case 0x71:
		case 0x75:
		case 0x87:
		case 0x88:
			dspArchType = DAT_FIRST;
			ret = DRT_64K; 
			break;
		case 0x76:
		case 0x83:
		case 0x92:
			dspArchType = DAT_FIRST;
			ret = DRT_128K;
			break;
		case 0x85:
			dspArchType = DAT_SECOND;
			ret = DRT_256K; 
			break;
		case 0x86:
			dspArchType = DAT_SECOND;
			ret = DRT_128K;
			break;
		case 0x89:
		case 0x90:
			dspArchType = DAT_SECOND;
			ret = DRT_128K;
			break;
		case 0x67:
			dspArchType = DAT_FIRST;
			ret = DRT_32K;
			break;
		default:
			ret = DRT_Unknow;
			break;
		}
		if (ret != DRT_Unknow)
		{
			*pChipID = dspID;
			*pAsicArchType = dspArchType;
			return ret;
		}
	}
	return ret;
}

BOOL XU_CustomReadFromSensor(BYTE slaveID, USHORT addr, BYTE addrByteNum, USHORT *pData, BYTE dataByteNum,  bool pollSCL)
{
	if (dataByteNum == 0)
		return FALSE;

	if (addrByteNum > 2) 
		addrByteNum = 2;
	if (dataByteNum > 2) 
		dataByteNum = 2;

	printf("test 1\n");
	USHORT dataBuffer = 0;
	if (!XU_CustomWriteToSensor(slaveID, addr, addrByteNum, dataBuffer, 0, pollSCL))
		return FALSE;

	printf("test 2\n");

	BYTE status;
	if (pollSCL){
		XU_ReadFromASIC(i2cMode, &status);
		XU_WriteToASIC(i2cMode, status | 0x03);
	}else{
		if (!XU_WriteToASIC(i2cMode, 0x01))
			return FALSE;
	}
	
	if (!XU_WriteToASIC(i2cSclSelOD, 0x01))	
		return FALSE;

	BYTE I2C_Speed = 0;
	if (!XU_ReadFromASIC(i2cDev, &I2C_Speed))
		return FALSE;
	if (I2C_Speed & 0x01){
		if (!XU_WriteToASIC(i2cDev, 0x83 | (dataByteNum << 4)))
			return FALSE;
	}
	else{
		if (!XU_WriteToASIC(i2cDev, 0x82 | (dataByteNum << 4)))
			return FALSE;
	}

	if (!XU_WriteToASIC(i2cSlaveID, slaveID))
		return FALSE;

	long i2cDataStartAddr = i2cDataArrStartAddr;
	while(i2cDataStartAddr < (i2cDataArrStartAddr + 5)){
		if (!XU_WriteToASIC(i2cDataStartAddr++, 0x00))
			return FALSE;
	}

	if (!XU_WriteToASIC(i2cTrg, 0x10))	
		return FALSE;

	BYTE value = 0x00;
	int i;
	for (i = 0; (i < 10) && !(value & 0x04); ++i)
	{
		if (!XU_ReadFromASIC(i2cDev, &value))
			return FALSE;
		usleep(1000);
	}

	if ((value & 0x0C) != 0x04){
		return FALSE;
	}

	if (!XU_ReadFromASIC(i2cDataArrStartAddr, &value))
		return FALSE;
	if (!XU_ReadFromASIC(i2cDataArrStartAddr + 1, &value))
		return FALSE;
	if (!XU_ReadFromASIC(i2cDataArrStartAddr + 2, &value))
		return FALSE;
	if (!XU_ReadFromASIC(i2cDataArrStartAddr + 3, &value))
		return FALSE;
	if (dataByteNum == 2)
		*pData = (USHORT)value << 8;
	if (!XU_ReadFromASIC(i2cDataArrStartAddr + 4, &value))
		return FALSE;
	if (dataByteNum == 2)
		*pData |= value;
	else if (dataByteNum == 1)
		*pData = value;

    return TRUE;
}

BOOL XU_CustomWriteToSensor(BYTE slaveID, USHORT addr, BYTE addrByteNum, USHORT data, BYTE dataByteNum, bool pollSCL)
{
	BYTE status;

	if(addrByteNum > 2)
		addrByteNum = 2;
	if(dataByteNum > 2)
		dataByteNum = 2;

	if (pollSCL)
	{
		XU_ReadFromASIC(i2cMode, &status);
		XU_WriteToASIC(i2cMode, status | 0x03);
	}else{
		if (!XU_WriteToASIC(i2cMode, 0x01))
			return FALSE;
	}

	if (!XU_WriteToASIC(i2cSclSelOD, 0x01))
		return FALSE;

	BYTE I2C_Speed = 0;
	if (!XU_ReadFromASIC(i2cDev, &I2C_Speed))
		return FALSE;
	if (I2C_Speed & 0x01){
		if (!XU_WriteToASIC(i2cDev, 0x81 | ((addrByteNum + dataByteNum) << 4)))	// I2C_DEV=1, I2C_SEL_RD=0(W)
			return FALSE;
	}
	else{
		if (!XU_WriteToASIC(i2cDev, 0x80 | ((addrByteNum + dataByteNum) << 4)))	// I2C_DEV=1, I2C_SEL_RD=0(W)
			return FALSE;
	}

	if (!XU_WriteToASIC(i2cSlaveID, slaveID))
		return FALSE;

	USHORT i2c_data_addr = i2cDataArrStartAddr;
	if (addrByteNum > 1){
		if (!XU_WriteToASIC(i2c_data_addr++, (BYTE)(addr >> 8)))
			return FALSE;
		if (!XU_WriteToASIC(i2c_data_addr++, (BYTE)addr))
			return FALSE;
	}
	else{
		if (!XU_WriteToASIC(i2c_data_addr++, (BYTE)addr))
			return FALSE;
	}

	if (dataByteNum > 1)
	{
		if (!XU_WriteToASIC(i2c_data_addr++, (BYTE)(data >> 8)))
			return FALSE;
		if (!XU_WriteToASIC(i2c_data_addr++, (BYTE)data))
			return FALSE;
	}
	else{
		if (!XU_WriteToASIC(i2c_data_addr++, (BYTE)data))
			return FALSE;
	}

	while (i2c_data_addr < (i2cDataArrStartAddr + 5)){
		if (!XU_WriteToASIC(i2c_data_addr++, 0x00))
			return FALSE;
	}

	if (!XU_WriteToASIC(i2cTrg, 0x10))
		return FALSE;

	BYTE value = 0x00;
	int i;
	for (i = 0; (i < 10) && !(value & 0x04); ++i)
	{
		if (!XU_ReadFromASIC(i2cDev, &value))
			return FALSE;
		usleep(1000);
	}
printf("flat 4, value=%x, ret=%x\n", value, (value & 0x0C));
	if ((value & 0x0C) == 0x04)
		return TRUE;
	return FALSE;
}

BOOL ReadDataFromAsic(USHORT addr, LONG* value)
{
	BYTE val[4] = { 0 };
	XU_ReadFromASIC(addr, val);
	XU_ReadFromASIC(addr+1, val+1);
	XU_ReadFromASIC(addr+2, val+2);
	XU_ReadFromASIC(addr+3, val+3);
	*value = val[0] << 24 + val[1] << 16 + val[2] << 8 + val[0];
	return TRUE;
}

BOOL WriteDataToAsic(USHORT addr, LONG value)
{
	XU_WriteToASIC(addr, value >> 24);
	XU_WriteToASIC(addr+1, value >> 16);
	XU_WriteToASIC(addr+2, value >> 8);
	XU_WriteToASIC(addr+3, value);
	return TRUE;
}

BOOL IsSupportI2C64()
{
	return g_supportI2C64;
}

BOOL IsSupportXU6()
{
	return g_supportXU6;
}


BOOL SetSFControllerSCK()
{
	BYTE romVersion[8] = { 0 };
	if (!XU_GetAsicRomVersion(romVersion)) {
		return FALSE;
	}
	switch (uiRomID)
	{
	case ROM275V1:
	case ROM276V1:
	case ROM287:
	{
		BYTE byRegValue = 0;
		if (!XU_ReadFromASIC(0x100D, &byRegValue))
			return FALSE;
		byRegValue = (byRegValue & (~0x04));// Set bit 2 = 0
		if (!XU_WriteToASIC(0x100D, byRegValue))
			return FALSE;
		// SPEED = 1
		if (!XU_WriteToASIC(0x1090, 0x01))
			return FALSE;
	}
	break;
	case ROM288V1:
	case ROM286:
		if (!XU_WriteToASIC(0x8045, 4))
			return FALSE;
		// SPEED = 1
		if (!XU_WriteToASIC(0x8E10, 4))
			return FALSE;
		break;
	case ROM289:
	case ROM290:
		if (!XU_WriteToASIC(0x8045, 10))
			return FALSE;
		// SPEED = 1
		if (!XU_WriteToASIC(0x8E10, 1))
			return FALSE;
		break;
	default:
		break;
	}
	return TRUE;
}

void StopReadWrite_SFXU6()
{
	LONG startAddrValue = 0, endAddrValue = 0;
	ReadDataFromAsic(SF_XU64_RW_START_ADDR, startAddrValue);
	ReadDataFromAsic(SF_XU64_RW_END_ADDR, endAddrValue);
	if (startAddrValue != endAddrValue) {
		WriteDataToAsic(SF_XU64_RW_START_ADDR, 0);
		WriteDataToAsic(SF_XU64_RW_END_ADDR, 0);
		XU_WriteToASIC(SF_XU64_EU_ADDR, 0);
	}
}

BOOL SetReadInfo_SFXU6(LONG startAddr, LONG dataSize, LONG bufferSize)
{
	StopReadWrite_SFXU6();

	XU6Data theXUData = { 0 };
	theXUData.dwStartAddr = startAddr;
	theXUData.bySFLen = (BYTE)bufferSize;
	theXUData.dwEndAddr = startAddr + dataSize;
	theXUData.bySFReadCmd = 0x0B;
	theXUData.bySFOutMoeSel = 0x01;
	// Dummy Write for set read info.
	theXUData.byCmd = (2 << 6);

	return WriteDataToFlashXU6(0, (BYTE*)&theXUData, sizeof(theXUData));
}

BOOL SetWriteInfo_SFXU6(LONG startAddr, LONG dataSize, LONG bufferSize)
{
	StopReadWrite_SFXU6();

	XU6Data	 theXUData = { 0 };
	theXUData.dwStartAddr = startAddr;
	theXUData.bySFLen = (BYTE)bufferSize;
	theXUData.dwEndAddr = startAddr + dataSize;
	theXUData.bySFReadCmd = 0x0B;
	theXUData.bySFOutMoeSel = 0x01;

	// Normal Write for set write info.
	theXUData.byCmd = 0 << 6;

	return WriteDataToFlashXU6(0, (BYTE*)&theXUData, sizeof(theXUData));
}

BOOL ReadDataFormFlashXU6(LONG addr, BYTE pData[], BYTE dataLength)
{
	if(!pData || !dataLength || dataLength > DF_XU_DATA_SIZE_SF64) return FALSE;

	BYTE xu6Data[DF_XU_DATA_SIZE_SF64] = { 0 };

	__u8 xu_unit= g_curExtendUnitID; 
	__u8 xu_selector= XU_SONIX_SYS_ASIC_RW;
	__u16 xu_size= DF_XU_DATA_SIZE_SF64;

	int ret = 0;
	if ((ret=XU_Get_Cur(xu_unit, xu_selector, xu_size, xu6Data)) < 0) 
	{
		printf("read xu6 error %d\n",ret);
		return FALSE;
	}

	memcpy(pData, xu6Data, dataLength);
	return TRUE;
}

BOOL WriteDataToFlashXU6(LONG addr, BYTE pData[], BYTE dataLength)
{
	if(!pData || !dataLength || dataLength > DF_XU_DATA_SIZE_SF64) return FALSE;
	
	BYTE xu6Data[DF_XU_DATA_SIZE_SF64] = { 0 };
	memcpy(xu6Data, pData, dataLength);

	__u8 xu_unit= g_curExtendUnitID; 
	__u8 xu_selector= XU_SONIX_SYS_ASIC_RW;
	__u16 xu_size= DF_XU_DATA_SIZE_SF64;

	int ret = 0;
	if ((ret=XU_Set_Cur(xu_unit, xu_selector, xu_size, xu6Data)) < 0) 
	{
		printf("write xu6 error %d\n",ret);
		return FALSE;
	}
	return TRUE;
}


BOOL ReadFormSF_XU6(LONG addr, BYTE pData[], LONG len)
{
	BYTE temp[DF_XU_DATA_SIZE_SF64];
	LONG step = DF_XU_DATA_SIZE_SF64;
	LONG addrIndex = 0;
	LONG startAddr = addr;

	SetSFControllerSCK();
	SetReadInfo_SFXU6(startAddr, len, DF_XU_DATA_SIZE_SF64);

	LONG loop = len / step;
	LONG ram = len % step;

	for (LONG i = 0; i < loop; i++) {
		memset(&temp, 0xff, step);
		if (!ReadDataFormFlashXU6(startAddr, temp, step))
			return FALSE;
		memcpy(pData + addrIndex, temp, DF_XU_DATA_SIZE_SF64);
		addrIndex += step;
		startAddr += step;
	}
	if (ram > 0) {
		memset(&temp, 0xff, step);
		if (!ReadDataFormFlashXU6(startAddr, temp, ram))
			return FALSE;
		memcpy(pData + addrIndex, temp, ram);
	}
	return TRUE;
}

BOOL WriteToSF_XU6(LONG addr, BYTE pData[], LONG len)
{
	BYTE temp[DF_XU_DATA_SIZE_SF64];
	LONG step = DF_XU_DATA_SIZE_SF64;
	LONG startAddr = addr;

	SetSFControllerSCK();
	SetWriteInfo_SFXU6(addr, len, DF_XU_DATA_SIZE_SF64);

	LONG loop = len / step;
	LONG ram = len % step;

	for (LONG i = 0; i < loop; i++){
		memcpy(temp, pData + i * step, step);
		if (!WriteDataToFlashXU6(startAddr, temp, step))
			return FALSE;
		startAddr += step;
	}
	if (ram > 0){
		memset(&temp, 0xFF, step);
		memcpy(temp, pData + (loop * step), ram);
		if (!WriteDataToFlashXU6(startAddr, temp, ram))
			return FALSE;
	}
	return TRUE;
}

BOOL EnableAsicRegisterBit(LONG addr, BYTE bit)
{
	BYTE bufs;
	BYTE bufd;
	if (!XU_ReadFromASIC(addr, &bufs))
		return false;

	bufd = bufs | (0x01 << bit);

	return XU_WriteToASIC(addr, bufd);
}

BOOL DisableAsicRegisterBit(LONG addr, BYTE bit)
{
	BYTE bufs;
	BYTE bufd;
	if (!XU_ReadFromASIC(addr, &bufs))
		return false;

	bufd = bufs & (~(0x01 << bit));

	return XU_WriteToASIC(addr, bufd);
}

BOOL SensorTwoWrite285(BYTE slaveID, USHORT addr, BYTE addrByteNum, BYTE* data, BYTE dataByteNum, bool pollSCL)
{
	USHORT i2c2_Status = 0x8620;
	USHORT i2c2_Trigger = 0x8621;
	USHORT i2c2_SlaveID = 0x8622;
	USHORT i2c2_MRDSA = 0x8628;

	BYTE status;
	if (pollSCL){
		XU_ReadFromASIC(i2cMode, &status);
		XU_WriteToASIC(i2cMode, status | 0x03);
	}

	if (!XU_DisableAsicRegisterBit(i2c2_Status, 7)) // Select Master mode
		return FALSE;

	if (!XU_WriteToASIC(i2c2_SlaveID, slaveID))   // Set Slave ID
		return FALSE;

	BYTE I2C_Speed = 0;
	if (!XU_ReadFromASIC(i2c2_Status, &I2C_Speed))
		return FALSE;
	if (I2C_Speed & 0x01) {
		if (!XU_WriteToASIC(i2c2_Status, 0x05 | ((addrByteNum + dataByteNum) << 4)))	//I2C2_MODE=0 SCL2_SEL_OD=1  I2C2_SEL_RD=0 I2C2_HIGH=1(400Kbps)  
			return FALSE;
	}
	else {
		if (!XU_WriteToASIC(i2c2_Status, 0x04 | ((addrByteNum + dataByteNum) << 4)))	//I2C2_MODE=0 SCL2_SEL_OD=1  I2C2_SEL_RD=0 I2C2_HIGH=0(100Kbps) 
			return FALSE;
	}

	
	//write addr
	USHORT i2c_data_addr = i2c2_MRDSA;
	if (addrByteNum > 1) {
		if (!XU_WriteToASIC(i2c_data_addr++, (BYTE)(addr >> 8)))
			return FALSE;
	}
	if (!XU_WriteToASIC(i2c_data_addr++, (BYTE)addr))
		return FALSE;

	// write data
	for(int i = 0; i < dataByteNum; i++){
			if (!XU_WriteToASIC(i2c_data_addr++, (BYTE)(data[i])))
			return FALSE;
	}
	while (i2c_data_addr < (i2c2_MRDSA + 5)) {
		if (!XU_WriteToASIC(i2c_data_addr++, 0x00))
			return FALSE;
	}

	//trigger to start i2c interface read/write
	if (!XU_WriteToASIC(i2c2_Trigger, 0x01))	// I2C_RW_TRG=1
		return FALSE;

	// wait I2C ready (time-out 10ms)
	BYTE value = 0x00;
	int i = 0;
	for (i = 0; (i < 10) && !(value & 0x02); ++i)
	{
		if (!XU_ReadFromASIC(i2c2_Trigger, &value))
			return FALSE;
		usleep(1000);
	}
	if (i >= 10)
		return FALSE;

	if (!XU_ReadFromASIC(i2c2_Status, &status))
		return FALSE;
	if (status & 0x08) //I2C2_ERR
		return FALSE;
		
	return TRUE;
}

BOOL SensorTwoWrite(BYTE slaveID, USHORT addr, BYTE addrByteNum, BYTE* data, BYTE dataByteNum, bool pollSCL)
{
	USHORT i2c2_Status = 0x8620;
	USHORT i2c2_Trigger = 0x862a;
	USHORT i2c2_SlaveID = 0x8621;
	USHORT i2c2_MRDSA = 0x8622;
	BYTE I2C2_CtrlBufSize = 8;
	if (g_chipID != 0x86) {
		i2c2_Status = 0x8640;
		i2c2_Trigger = 0x864a;
		i2c2_SlaveID = 0x8641;
		i2c2_MRDSA = 0x8642;
	}

	EnableAsicRegisterBit(i2c2_Status, 0);    //I2C2_MODE	  	= 1;
	DisableAsicRegisterBit(i2c2_Status, 4);   //I2C2_SEL_RD	  	= 0;

	if (!XU_WriteToASIC(i2c2_SlaveID, slaveID))   // Set Slave ID
		return false;

	BYTE wrDataBuf[257];
	memset(wrDataBuf, 0, sizeof(wrDataBuf));
	memcpy(wrDataBuf, &addr, addrByteNum);
	memcpy(wrDataBuf + addrByteNum, data, dataByteNum);

	BYTE tempByteNum = addrByteNum + dataByteNum;
	BYTE i, j;
	BYTE I2C2_WrLens = 0;
	BYTE I2C2_WrIdx = 0;
	BYTE I2C2_WrStopBit = 0;
	USHORT i2c_data_addr = i2c2_MRDSA;
	while (tempByteNum > 0) {
		if (tempByteNum > I2C2_CtrlBufSize) {
			j = I2C2_CtrlBufSize;
			I2C2_WrLens += I2C2_CtrlBufSize;
			tempByteNum -= I2C2_CtrlBufSize;
		}else {
			j = tempByteNum;
			tempByteNum = 0;
			I2C2_WrStopBit = 0x10;
		}

		i2c_data_addr = i2c2_MRDSA;
		for (i = 0; i < j; i++){
			if (!XU_WriteToASIC(i2c_data_addr++, wrDataBuf[I2C2_WrIdx++]))
				return false;
		}
		while (i2c_data_addr < (i2c2_MRDSA + I2C2_CtrlBufSize)) {
			if (!XU_WriteToASIC(i2c_data_addr++, 0x00))
				return false;
		}
		//trigger to start i2c interface read/write
		if (!XU_WriteToASIC(i2c2_Trigger, ((j - 1) << 5) | I2C2_WrStopBit | 0x01))	// I2C_RW_TRG=1
			return false;

		// wait I2C ready (time-out 10ms)
		BYTE value = 0x00;
		for (i = 0; (i < 10) && !(value & 0x40); ++i)
		{
			if (!XU_ReadFromASIC(i2c2_Status, &value))
				return false;
			usleep(1000);
		}
		if (i >= 10)
			return false;
		BYTE status;
		if (!XU_ReadFromASIC(i2c2_Status, &status))
			return false;
		if (status & 0x80) //I2C2_ERR
			return false;
	}

	return true;
}

BOOL SensorTwoRead285(BYTE slaveID, USHORT addr, BYTE addrByteNum, BYTE* pData, BYTE dataByteNum, bool pollSCL)
{
	USHORT i2c2_Status = 0x8620;
	USHORT i2c2_Trigger = 0x8621;
	USHORT i2c2_SlaveID = 0x8622;
	USHORT i2c2_MRDSA = 0x8628;   //MasterReadDataStartAddr

	if (dataByteNum == 0)
		return FALSE;

	if (addrByteNum > 2) addrByteNum = 2;
	if (dataByteNum > 2) dataByteNum = 2;

	// IIC dummy write
	USHORT dataBuffer = 0;
	if (!XU_CustomWriteToSensorTwo(slaveID, addr, addrByteNum, dataBuffer, 0, pollSCL))
		return FALSE;

	BYTE status;
	if (pollSCL)
	{
		XU_ReadFromASIC(i2cMode, &status);
		XU_WriteToASIC(i2cMode, status | 0x03);
	}

	if (!XU_DisableAsicRegisterBit(i2c2_Status, 7)) // Select Master mode
		return FALSE; 

	if (!XU_WriteToASIC(i2c2_SlaveID, slaveID))
		return FALSE;

	BYTE I2C_Speed = 0;
	if (!XU_ReadFromASIC(i2c2_Status, &I2C_Speed))
		return FALSE;
	if (I2C_Speed & 0x01) {
		if (!XU_WriteToASIC(i2c2_Status, 0x07 | (dataByteNum << 4)))	//I2C2_MODE=0 SCL2_SEL_OD=1  I2C2_SEL_RD=1 I2C2_HIGH=1(400Kbps)  
			return FALSE;
	}
	else {
		if (!XU_WriteToASIC(i2c2_Status, 0x06 | (dataByteNum << 4)))	//I2C2_MODE=0 SCL2_SEL_OD=1  I2C2_SEL_RD=1 I2C2_HIGH=0(100Kbps) 
			return FALSE;
	}

	for (USHORT addr = i2c2_MRDSA; addr < i2c2_MRDSA + 5; addr++)
	{
		if (!XU_WriteToASIC(addr, 0x00))
			return FALSE;
	}
	if (!XU_WriteToASIC(i2c2_Trigger, 0x01))	// I2C_RW_TRG=1
		return FALSE;

	// wait I2C ready (time-out 10ms)
	BYTE value = 0x00;
	int i = 0;
	for (i = 0; (i < 10) && !(value & 0x02); ++i)
	{
		if (!XU_ReadFromASIC(i2c2_Trigger, &value))
			return FALSE;
		usleep(1000);
	}
	if (i >= 10)
		return FALSE;
	if (!XU_ReadFromASIC(i2c2_Status, &status))
		return FALSE;
	if (status & 0x08) //I2C2_ERR
		return FALSE;

	for(i = 0; i < dataByteNum; i++){
		if (!XU_ReadFromASIC(i2c2_MRDSA + i, &value))
			return FALSE;
		pData[i] = value;
	}
	return TRUE;
}

BOOL SensorTwoRead(BYTE slaveID, USHORT addr, BYTE addrByteNum, BYTE* pData, BYTE dataByteNum, bool pollSCL)
{
	USHORT i2c2_Status = 0x8620;
	USHORT i2c2_Trigger = 0x862a;
	USHORT i2c2_SlaveID = 0x8621;
	USHORT i2c2_MRDSA = 0x8622; //MasterReadDataStartAddr
	BYTE I2C2_CtrlBufSize = 8;
	if (g_chipID != 0x86) {
		i2c2_Status = 0x8640;
		i2c2_Trigger = 0x864a;
		i2c2_SlaveID = 0x8641;
		i2c2_MRDSA = 0x8642;
	}

	if (dataByteNum == 0)
		return false;

	// IIC dummy write
	USHORT dataBuffer = 0;
	if (!SensorTwoWrite(slaveID, addr, addrByteNum, pData, 0, pollSCL))
		return false;

	EnableAsicRegisterBit(i2c2_Status, 0);    //I2C2_MODE	  	= 1;
	EnableAsicRegisterBit(i2c2_Status, 4);   //I2C2_SEL_RD	  	= 1;

	if (!XU_WriteToASIC(i2c2_SlaveID, slaveID))
		return false;

	BYTE tempByteNum = dataByteNum;
	BYTE i, j;
	BYTE I2C2_RdLens = 0;
	BYTE I2C2_RdIdx = 0;
	BYTE I2C2_RdStopBit = 0;
	while (tempByteNum > 0) {
		if (tempByteNum > I2C2_CtrlBufSize) {
			j = I2C2_CtrlBufSize;
			I2C2_RdLens += I2C2_CtrlBufSize;
			tempByteNum -= I2C2_CtrlBufSize;
		}
		else {
			j = tempByteNum;
			tempByteNum = 0;
			I2C2_RdStopBit = 0x10;
		}
		for (USHORT addr = i2c2_MRDSA; addr < (i2c2_MRDSA + I2C2_CtrlBufSize); addr++)
		{
			if (!XU_WriteToASIC(addr, 0x00))
				return false;
		}

		if (!XU_WriteToASIC(i2c2_Trigger, ((j - 1) << 5) | I2C2_RdStopBit | 0x01))	// I2C_RW_TRG=1
			return false;

		// wait I2C ready (time-out 10ms)
		BYTE value = 0x00;
		int i = 0;
		for (i = 0; (i < 10) && !(value & 0x40); ++i)
		{
			if (!XU_ReadFromASIC(i2c2_Status, &value))
				return false;
			usleep(1000);
		}
		if (i >= 10)
			return false;
		BYTE status;
		if (!XU_ReadFromASIC(i2c2_Status, &status))
			return false;
		if (status & 0x80) //I2C2_ERR
			return false;

		for (i = 0; i < j; i++) {
			if (!XU_ReadFromASIC(i2c2_MRDSA + i, &value))
				return false;
			pData[I2C2_RdIdx++] = value;
		}
	}
	return true;
}

BOOL XU_CustomReadFromSensorTwo(BYTE slaveID, USHORT addr, BYTE addrByteNum, BYTE *pData, BYTE dataByteNum, bool pollSCL)
{
	bool r = false;
	switch (g_chipID) {
	case 0x85:
		r = SensorTwoRead285(slaveID, addr, addrByteNum, pData, dataByteNum, pollSCL);
		break;
	case 0x86:
	case 0x89:
	case 0x90:
		r = SensorTwoRead(slaveID, addr, addrByteNum, pData, dataByteNum, pollSCL);
		break;
	default:
		break;
	}
	return r;
}

BOOL XU_CustomWriteToSensorTwo(BYTE slaveID, USHORT addr, BYTE addrByteNum, BYTE* data, BYTE dataByteNum, bool pollSCL)
{
	bool r = false;
	switch (g_chipID) {
	case 0x85:
		r = SensorTwoWrite285(slaveID, addr, addrByteNum, data, dataByteNum, pollSCL);
		break;
	case 0x86:
	case 0x89:
	case 0x90:
		r = SensorTwoWrite(slaveID, addr, addrByteNum, data, dataByteNum, pollSCL);
		break;
	default:
		break;
	}
	return r;
}

BOOL XU_ReadDataFormFlash(LONG addr, BYTE pData[], BYTE dataLen)
{
	__u8 ctrldata[11]={0};
	__u8 xu_unit= g_curExtendUnitID; 
	__u8 xu_selector= 0x03;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	xu_data[0] = (BYTE)((addr << 8) >> 8);
	xu_data[1] = (BYTE)(addr >> 8);
	BYTE temp;
	if (addr < 0x10000) 
		temp = 0x88;
	else if (addr < 0x20000) 
		temp = 0x98;
	else if (addr < 0x30000) 
		temp = 0xA8;
	else 
		temp = 0xB8;
	xu_data[2]  = (temp & 0xF0) | dataLen;

	if (XU_Set_Cur(xu_unit, xu_selector, xu_size, xu_data) < 0) 
		return FALSE;

	//memset(xu_data, 0, xu_size);
    if (XU_Get_Cur(xu_unit, xu_selector, xu_size, xu_data) < 0) 
    	return FALSE;

	memcpy(pData, xu_data+3, dataLen);	
	return TRUE;
}

BOOL XU_WriteDataToFlash(LONG addr, BYTE pData[], BYTE dataLen)
{
	__u8 ctrldata[11]={0};
	__u8 xu_unit= g_curExtendUnitID; 
	__u8 xu_selector= 0x03;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if(dataLen > 8)
		dataLen = 8;

	xu_data[0] = (BYTE)((addr << 8) >> 8);
	xu_data[1] = (BYTE)(addr >> 8);
	if (addr < 0x10000)
		xu_data[2] = 0x08;
	else if (addr < 0x20000)
		xu_data[2] = 0x18;
	else if (addr < 0x30000)
		xu_data[2] = 0x28;
	else
		xu_data[2]= 0x38;
	xu_data[2] &= 0xf0;
	xu_data[2] |= dataLen;

	memcpy(xu_data + 3, pData, dataLen);
	if (XU_Set_Cur(xu_unit, xu_selector, xu_size, xu_data) < 0) 
		return FALSE;

    return TRUE;
}

BOOL XU_ReadFormSF(LONG addr, BYTE pData[], LONG len)
{
	BYTE tempData[8];
	LONG startAddr = addr;
	LONG loop = len / 8;
	LONG ram = len % 8;
	LONG addrIndex = 0;
	for (LONG i = 0; i < loop; i++){
		memset(&tempData, 0xff, 8);
		if(!XU_ReadDataFormFlash(startAddr, tempData, 8))
			return FALSE;

		memcpy(pData + addrIndex, tempData, 8);
		addrIndex += 8;
		startAddr += 8;
	}
	if (ram > 0){
		memset(&tempData, 0xff, 8);
		if (!XU_ReadDataFormFlash(startAddr, tempData, ram))
			return FALSE;

		memcpy(pData + addrIndex, tempData, ram);
	}

	return TRUE;
}

BOOL XU_WriteToSF(LONG addr, BYTE pData[], LONG len)
{
	LONG startAddr = addr;
	LONG loop = len / 8;
	LONG ram = len % 8;
	BYTE tempData[8];
	for (LONG i = 0; i < loop; i++)
	{
		memcpy(tempData, pData + i * 8, 8);
		if (!XU_WriteDataToFlash(startAddr, tempData, 8))
			return FALSE;
		startAddr += 8;
	}
	if (ram > 0)
	{
		memset(&tempData, 0xFF, 8);
		memcpy(tempData, pData + (loop * 8), ram);
		if (!XU_WriteDataToFlash(startAddr, tempData, ram))
			return FALSE;
	}
	return TRUE;
}

BYTE XU_GetUVCExtendUnitID()
{
	BYTE chipID;
	DSP_ARCH_TYPE dspArchType;

	g_curExtendUnitID = 0x03;
	DSP_ROM_TYPE romType = XU_GetChipRomType(&chipID, &dspArchType);
	if(DRT_Unknow != romType){
		return 0x3;
	 }

	g_curExtendUnitID = 0x04;
	romType = XU_GetChipRomType(&chipID, &dspArchType);
	if(DRT_Unknow != romType)
		return 0x4;
	
	printf("Get uvc extend unit id fail\n");
	return 0;
}

BOOL XU_ReadFromROM(LONG addr, BYTE data[])
{
	unsigned int remain = 0, rd_size, offset;
	__u8 ctrldata[11]={0};
	__u8 xu_unit= g_curExtendUnitID; 
	__u8 xu_selector= 0x04;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	xu_data[0] = (BYTE)((addr << 8) >> 8);
	xu_data[1] = (BYTE)(addr >> 8);
	xu_data[2]  = 8;

	if (XU_Set_Cur(xu_unit, xu_selector, xu_size, xu_data) < 0) 
		return FALSE;

    if (XU_Get_Cur(xu_unit, xu_selector, xu_size, xu_data) < 0) 
    	return FALSE;

	memcpy(data, xu_data + 3, 8);	

	return TRUE;
}

BOOL XU_DefGetAsicRomVersion(BYTE data[])
{
	BYTE romString[8];
	int i = 0, j = 0;
	for (i = 0; i < ROMSTRADDRCNT; i++)
	{
		XU_ReadFromROM(RomStringAddr[i], romString);
		for (j = 0; j < ROMCOUNT; j++)
		{
			if (RomInfo[j].RomStringAddr == RomStringAddr[i])
			{
				if (RomInfo[j].IsNewestVer)
				{
					if (memcmp(romString, RomInfo[j].RomString, 4) == 0 &&
						romString[5] >= RomInfo[j].RomString[5])
					{
						memcpy(data, romString, 8);
						uiRomID = j;	// shawn 2010/05/14 add
						return TRUE;
					}
				}
				else
				{
					if (memcmp(romString, RomInfo[j].RomString, 4) == 0 &&	// shawn 2009/06/10 modify
						romString[5] == RomInfo[j].RomString[5])				// shawn 2010/04/12 only compare 6 bytes
					{
						memcpy(data, romString, 8);
						uiRomID = j;	// shawn 2010/05/14 add
						return TRUE;
					}
				}
			}

		}
	}
	data = 0;
	return FALSE;
}

BOOL XU_GetAsicRomVersion(BYTE data[])
{
	if (TRUE == XU_DefGetAsicRomVersion(data)){
		if (uiRomID == ROM276V1){
			BYTE byTmp1 = 0;
			BYTE byTmp2 = 0;
			XU_ReadFromASIC(0x1185, &byTmp1);
			byTmp1 |= 0x70;
			XU_WriteToASIC(0x1185, byTmp1);
			XU_ReadFromASIC(0x1185, &byTmp2);
			if ((byTmp2 & 0x70) == (byTmp1 & 0x70)){
				data[4] = 0x31;
			}
		}
		if (uiRomID == ROM288V1){
			BYTE byTmp1 = 0;
			BYTE byTmp2 = 0;
			XU_ReadFromASIC(0x101f, &byTmp1);
			if (byTmp1 == 0x89){
				data[2] = 0x39;
				uiRomID = ROM289V1;
			}
			else{
				byTmp1 = 0;
				XU_ReadFromASIC(0x1007, &byTmp1);
				byTmp1 &= 0xDF;
				XU_ReadFromASIC(0x1006, &byTmp2);
				byTmp2 |= 0x20;
				XU_WriteToASIC(0x1007, byTmp1);
				XU_WriteToASIC(0x1006, byTmp2);
				byTmp1 = 0;
				XU_ReadFromASIC(0x1005, &byTmp1);
				if ((byTmp1 & 0x20) == 0x20){
					data[4] = 0x31;
				}
				else{
					byTmp2 &= 0xDF;
					XU_WriteToASIC(0x1006, byTmp2);
				}
			}
		}
		if (uiRomID == ROM271V1){
			BYTE byTmp1 = 0;
			BYTE byTmp2 = 0;
			BYTE byTmp3 = 0;
			XU_ReadFromASIC(0x100A, &byTmp1);
			byTmp2 = (byTmp1 & 0x10);
			byTmp1 &= 0xEF;
			XU_WriteToASIC(0x100A, byTmp1);
			XU_ReadFromASIC(0x101F, &byTmp3);
			byTmp1 |= byTmp2;
			XU_WriteToASIC(0x100A, byTmp1);
			if (byTmp3 == 0x70){
				data[2] = 0x30;
				uiRomID = ROM270V1;
			}
			byTmp1 = 0;
			XU_ReadFromASIC(0x1007, &byTmp1);
			byTmp1 &= 0xF7;
			byTmp2 = 0;
			XU_ReadFromASIC(0x1006, &byTmp2);
			byTmp2 |= 0x08;
			XU_WriteToASIC(0x1007, byTmp1);
			XU_WriteToASIC(0x1006, byTmp2);
			byTmp1 = 0;
			XU_ReadFromASIC(0x1005, &byTmp1);
			if ((byTmp1 & 0x08) == 0x08){
				data[4] = 0x31;	// 270M, 271M
				byTmp2 &= 0xF7;
				XU_WriteToASIC(0x1006, byTmp2);
			}
			else{
				byTmp1 = 0;
				XU_ReadFromASIC(0x1007, &byTmp1);
				byTmp1 &= 0xEF;
				byTmp2 = 0;
				XU_ReadFromASIC(0x1006, &byTmp2);
				byTmp2 |= 0x10;
				XU_WriteToASIC(0x1007, byTmp1);
				XU_WriteToASIC(0x1006, byTmp2);
				byTmp1 = 0;
				XU_ReadFromASIC(0x1005, &byTmp1);

				if ((byTmp1 & 0x10) == 0x10){
					// 270A, 271A
					data[4] = 0x30;
				}
				else{
					if (uiRomID == ROM270V1){
						data[4] = 0x32;	// 270B
						byTmp2 &= 0xEF;
						XU_WriteToASIC(0x1006, byTmp2);
					}
				}
			}
		}
		if (uiRomID == ROM281V1){
			BYTE byTmp1 = 0;
			BYTE byTmp2 = 0;
			BYTE byTmp3 = 0;
			XU_ReadFromASIC(0x100A, &byTmp1);
			byTmp2 = (byTmp1 & 0x10);
			byTmp1 &= 0xEF;
			XU_WriteToASIC(0x100A, byTmp1);
			XU_ReadFromASIC(0x101F, &byTmp3);
			byTmp1 |= byTmp2;
			XU_WriteToASIC(0x100A, byTmp1);
			if (byTmp3 == 0x80){
				data[2] = 0x30;
				uiRomID = ROM280V1;
			}
			byTmp1 = 0;
			XU_ReadFromASIC(0x1007, &byTmp1);
			byTmp1 &= 0xF7;
			byTmp2 = 0;
			XU_ReadFromASIC(0x1006, &byTmp2);
			byTmp2 |= 0x08;
			XU_WriteToASIC(0x1007, byTmp1);
			XU_WriteToASIC(0x1006, byTmp2);
			byTmp1 = 0;
			XU_ReadFromASIC(0x1005, &byTmp1);
			if ((byTmp1 & 0x08) == 0x08){
				data[4] = 0x31;	// 280M, 281M
				byTmp2 &= 0xF7;
				XU_WriteToASIC(0x1006, byTmp2);
			}
			else{
				data[4] = 0x30;	// 280A, 281A
			}
		}
		return TRUE;
	}
	data = 0;
	return FALSE;
}

BOOL XU_EnableAsicRegisterBit(LONG addr, BYTE bit)
{
	BYTE bufs;
	BYTE bufd;
	if (!XU_ReadFromASIC(addr, &bufs))
		return FALSE;
	switch (bit)
	{
	case 0:
		bufd = bufs | 0x01;
		break;
	case 1:
		bufd = bufs | 0x02;
		break;
	case 2:
		bufd = bufs | 0x04;
		break;
	case 3:
		bufd = bufs | 0x08;
		break;
	case 4:
		bufd = bufs | 0x10;
		break;
	case 5:
		bufd = bufs | 0x20;
		break;
	case 6:
		bufd = bufs | 0x40;
		break;
	case 7:
		bufd = bufs | 0x80;
		break;
	default:
		break;
	}
	return XU_WriteToASIC(addr, bufd);
}

BOOL XU_DisableAsicRegisterBit(LONG addr, BYTE bit)
{
	BYTE bufs;
	BYTE bufd;

	if (!XU_ReadFromASIC(addr, &bufs))
		return FALSE;
	switch (bit)
	{
	case 0:
		bufd = bufs & 0xfe;
		break;
	case 1:
		bufd = bufs & 0xfd;
		break;
	case 2:
		bufd = bufs & 0xfb;
		break;
	case 3:
		bufd = bufs & 0xf7;
		break;
	case 4:
		bufd = bufs & 0xef;
		break;
	case 5:
		bufd = bufs & 0xdf;
		break;
	case 6:
		bufd = bufs & 0xbf;
		break;
	case 7:
		bufd = bufs & 0x7f;
		break;
	default:
		break;
	}
	return XU_WriteToASIC(addr, bufd);
}

BOOL XU_Read(unsigned char pData[], unsigned int length, BYTE unitID, BYTE cs)
{
	int ret = 0;
	__u8 xu_unit= unitID; 
	__u8 xu_selector= cs;
	__u16 xu_size= length;
	__u8 *xu_data= pData;

	if ((ret=XU_Get_Cur(xu_unit, xu_selector, xu_size, xu_data)) < 0) {
		return FALSE;
	}
	if(ret < 0)
		return FALSE;
	return TRUE;
}

BOOL XU_Write(unsigned char pData[], unsigned int length,  BYTE unitID, BYTE cs)
{
	int ret = 0;
	__u8 xu_unit= unitID; 
	__u8 xu_selector= cs;
	__u16 xu_size= length;
	__u8 *xu_data= pData;
	
	if ((ret=XU_Set_Cur(xu_unit, xu_selector, xu_size, xu_data)) < 0) {
		return FALSE;	
	}
	return TRUE;
}

void XU_ReadSFID(BYTE cmd, BYTE dummyNum, BYTE devIdNum)
{
	LONG data;
	data = 0x1;
	XU_WriteToASIC(0x1080, data);
	data = 0x0;
	XU_WriteToASIC(0x1091, data);
	data = cmd;
	XU_WriteToASIC(0x1082, data);
	data = 0x01;
	XU_WriteToASIC(0x1081, data);
	XU_SFWaitReady();

	while (dummyNum > 0)
	{
		data = 0x00;
		XU_WriteToASIC(0x1082, data);
		data = 0x01;
		XU_WriteToASIC(0x1081, data);
		XU_SFWaitReady();
		dummyNum--;
	}

	data = 0x0;
	XU_WriteToASIC(0x1083, data);
	data = 0x02;
	XU_WriteToASIC(0x1081, data);
	XU_SFWaitReady();
	XU_ReadFromASIC(0x1083, &sfManufactureID);
	if (sfManufactureID == SF_MFRID_CONT)
	{
		data = 0x0;
		XU_WriteToASIC(0x1083, data);
		data = 0x02;
		XU_WriteToASIC(0x1081, data);
		XU_SFWaitReady();
		XU_ReadFromASIC(0x1083, &sfManufactureID);
	}

	data = 0x0;
	XU_WriteToASIC(0x1083, data);
	data = 0x02;
	XU_WriteToASIC(0x1081, data);
	XU_SFWaitReady();
	XU_ReadFromASIC(0x1083, &sfDeviceID1);
	if (devIdNum == 2)
	{
		data = 0x0;
		XU_WriteToASIC(0x1083, data);
		data = 0x02;
		XU_WriteToASIC(0x1081, data);
		XU_SFWaitReady();
		XU_ReadFromASIC(0x1083, &sfDeviceID2);
	}
	else
		sfDeviceID2 = 0xFF;

	data = 0x00;
	XU_WriteToASIC(0x1080, data);
}

SERIAL_FLASH_TYPE XU_SerialFlashSearch()
{
	BYTE i, ubID_Num;
	BYTE ubSFType = SF_UNKNOW;
	ubID_Num = ubSFLib_GetIDSize();
	for (i = 1; i<ubID_Num; i++)
	{
		if (cbSFLib_ID[i][SFCMD_INFO_MFR] == sfManufactureID &&
			(cbSFLib_ID[i][SFCMD_INFO_DEVID1] == sfDeviceID1 || cbSFLib_ID[i][SFCMD_INFO_DEVID1] == 0xFF) &&
			(cbSFLib_ID[i][SFCMD_INFO_DEVID2] == sfDeviceID2 || cbSFLib_ID[i][SFCMD_INFO_DEVID2] == 0xFF))
		{
			break;
		}
	}
	if (i == ubID_Num)
		i = 0;
	ubSFType = cbSFLib_ID[i][SFCMD_INFO_TYPE];
	return (SERIAL_FLASH_TYPE)ubSFType;
}

SERIAL_FLASH_TYPE XU_SerialFlashIdentify()
{
	// MXIC-like series
	XU_ReadSFID(SFCMD_RDID_MXIC, 0, 2);
	SERIAL_FLASH_TYPE sfType = XU_SerialFlashSearch();
	if (sfType != SF_UNKNOW)
		goto sfIndetifyExit;

	// Atmel AT25F series
	XU_ReadSFID(SFCMD_RDID_AT25F, 0, 1);
	sfType = XU_SerialFlashSearch();
	if (sfType != SF_UNKNOW)
		goto sfIndetifyExit;

	// SST/Winbond/Other MXIC-like
	XU_ReadSFID(SFCMD_REMS_SST, 3, 1);
	sfType = XU_SerialFlashSearch();
	if (sfType != SF_UNKNOW)
		goto sfIndetifyExit;
	
	// ST/PMC
	XU_ReadSFID(SFCMD_RES_ST, 3, 1);
	sfType = XU_SerialFlashSearch();
	if (sfType != SF_UNKNOW)
		goto sfIndetifyExit;
	
sfIndetifyExit:
	return sfType;
}

BOOL XU_GetSerialFlashType(SERIAL_FLASH_TYPE *sft, bool check)
{
	*sft = XU_SerialFlashIdentify();
	BYTE sfType = 0;
	if(check){
		if(*sft == SFT_UNKNOW){
			BYTE romVersion[8] = { 0 };
			if (!XU_GetAsicRomVersion(romVersion)){
				return FALSE;
			}
			BYTE chipID = 0;
			DSP_ARCH_TYPE archType = DAT_UNKNOW;
			if(DRT_Unknow == XU_GetChipRomType(&chipID, &archType)){
				return FALSE;
			}
			if (!XU_ReadFromASIC(RomInfo[uiRomID].SFTypeAddr, &sfType))
				return FALSE;
			if (sfType != SFT_UNKNOW && sfType < SFT_COUNT)
				*sft = (SERIAL_FLASH_TYPE)sfType;
			else
				return FALSE;
		}
	}
	return TRUE;
}

BOOL XU_DisableSerialFlashWriteProtect(SERIAL_FLASH_TYPE sft)
{
	if (sft == SFT_UNKNOW)
	{
		return FALSE;
	}

	BYTE romVersion[8] = { 0 };
	if (TRUE != XU_GetAsicRomVersion(romVersion))
	{
		return FALSE;
	}
	
	BYTE chipID;
	DSP_ARCH_TYPE dspArchType;
	DSP_ROM_TYPE drt = XU_GetChipRomType(&chipID, &dspArchType);
	if (DRT_Unknow == drt)
	{
		return FALSE;
	}

	//setp 1 : disable hardware wirete protect
	BYTE data[2] = { 0 };
	if (drt == DRT_64K)
	{
		XU_ReadFromASIC(gpioOEAddr, &data[1]);
		data[1] = data[1] | 0x1F;
		XU_WriteToASIC(gpioOEAddr, data[1]);
		XU_ReadFromASIC(gpioOEAddr, &data[1]);

		XU_ReadFromASIC(gpioOutputAddr, &data[0]);
		data[0] = data[0] | 0x1F;
		XU_WriteToASIC(gpioOutputAddr, data[0]);
		XU_ReadFromASIC(gpioOutputAddr, &data[0]);
	}
	else
	{
		XU_ReadFromASIC(gpioOEAddr, &data[1]);
		data[1] = data[1] | 0xFF;
		XU_WriteToASIC(gpioOEAddr, data[1]);
		XU_ReadFromASIC(gpioOEAddr, &data[1]);

		XU_ReadFromASIC(gpioOutputAddr, &data[0]);
		data[0] = data[0] | 0xFF;
		XU_WriteToASIC(gpioOutputAddr, data[0]);
		XU_ReadFromASIC(gpioOutputAddr, &data[0]);
	}

	BYTE wpData = 0;
	switch (chipID)
	{
	case 0x16:
		XU_ReadFormSF(0x5834, &wpData, 1);
		break;
	case 0x33:
		XU_ReadFormSF(0x000f, &wpData, 1);
		break;
	case 0x32:
	case 0x76:
	case 0x67:		
	case 0x71:
	case 0x85:
	case 0x86:
	case 0x89:
	case 0x90:
	case 0x75:
		wpData = 0xFF;
		break;
	}
	

	BYTE byTmpAddr = 0;
	BYTE byMemType = 0;
	BYTE wpGPIO = 0;
	BOOL isNewWPVer = FALSE;
	USHORT wpAddr = 0xFFFF;
	BYTE sfWriteEnable = 0;
	BYTE sfWriteCommand = 0;

	if (RomInfo[uiRomID].IsCompactMode)	// shawn 2010/05/14 modify
	{
		XU_GetMemType(&byMemType);
	}

	wpGPIO = (wpData >> 4) & 0x7;
	if ((uiRomID != ROM283) && (uiRomID != ROM292) && (uiRomID != ROM275V2) && (uiRomID != ROM287)){
		if ((wpData & 0x0C) == 0x08){
			isNewWPVer = TRUE;
		}
	}
	if (isNewWPVer){
		if ((wpData & 0x03) == 0x02){
			wpData = 1;
		}
		else if ((wpData & 0x03) == 0x03){
			wpData = 2;
		}
	}
	else{
		wpData = wpData & 0x03;
	}
	if (RomInfo[uiRomID].IsDisSFWriteCmd)
	{
		if (!XU_ReadFromASIC(0x05F3, &sfWriteEnable)){
			return FALSE;
		}
		if (isNewWPVer)
		{
			if (!XU_WriteToASIC(0x05F3, cbSFLib_Cmd[ubSFLib_CmdID][SFCMD_IDX_WREN]))	// SF write enable
				return FALSE;
		}
		else
		{
			if (!XU_WriteToASIC(0x05F3, 0x06))	// SF write enable(ubSFWREN)
				return FALSE;
		}
		//// SF write enable(ubSFWREN)
		//if (!XU_WriteToASIC(0x05F3, 0x06)){
		//	return FALSE;
		//}
		// Memkey1 low
		if (!XU_WriteToASIC(0x05F8, 0x12))	{
			return FALSE;
		}
		// MemKey1 high
		if (!XU_WriteToASIC(0x05F9, 0x12))	{
			return FALSE;
		}
		//memkey2 low
		if (!XU_WriteToASIC(0x05FA, 0xED))  {
			return FALSE;
		}
		//memkey2 high
		if (!XU_WriteToASIC(0x05FB, 0xED))  {
			return FALSE;
		}
		//ubsfwrite
		if (!XU_ReadFromASIC(0x05F5, &sfWriteCommand))  {
			return FALSE;
		}
		if (isNewWPVer){
			//SF write command
			if (!XU_WriteToASIC(0x05F5, cbSFLib_Cmd[ubSFLib_CmdID][SFCMD_IDX_PP])){
				return FALSE;
			}
		}
		else{
			if (sft == SFT_SST){
				//SF write command for SST
				if (!XU_WriteToASIC(0x05F5, 0xAF)){
					return FALSE;
				}
			}
			else{
				if (!XU_WriteToASIC(0x05F5, 0x02)){
					return FALSE;
				}
			}
		}
	}

	if (RomInfo[uiRomID].IsCompactMode && byMemType == 2){
		return TRUE;
	}
	//disable write protect @ flash status register (bp0, bp1)
	if (!XU_WriteToASIC(sfModeAddr, 0x1)){
		return FALSE;
	}
	if (!XU_WriteToASIC(sfCSAddr, 0x0)){
		return FALSE;
	}
	if (!XU_WriteToASIC(sfWriteDataAddr, 0x06)){
		return FALSE;
	}
	if (!XU_WriteToASIC(sfReadWriteTriggerAddr, 0x1)){
		return FALSE;
	}
	if (!XU_SFWaitReady()){
		return FALSE;
	}
	if (!XU_WriteToASIC(sfCSAddr, 0x1)){
		return FALSE;
	}
	if (!XU_SFCMDreadStatus()){
		return FALSE;
	}
	if (sft == SFT_SST)
	{
		if (!XU_WriteToASIC(sfCSAddr, 0x0)){
			return FALSE;
		}
		// Enable-Write-Status-Register(EWSR)
		if (!XU_WriteToASIC(sfWriteDataAddr, 0x50)){
			return FALSE;
		}
		if (!XU_WriteToASIC(sfReadWriteTriggerAddr, 0x1)){
			return FALSE;
		}
		if (!XU_SFWaitReady()){
			return FALSE;
		}
		if (!XU_WriteToASIC(sfCSAddr, 0x1)){
			return FALSE;
		}
	}
	if (!XU_WriteToASIC(sfCSAddr, 0x0)){
		return FALSE;
	}
	if (!XU_WriteToASIC(sfWriteDataAddr, 0x1)){
		return FALSE;
	}
	if (!XU_WriteToASIC(sfReadWriteTriggerAddr, 0x1)){
		return FALSE;
	}
	if (!XU_SFWaitReady()){
		return FALSE;
	}
	if (!XU_WriteToASIC(sfWriteDataAddr, 0x0)){
		return FALSE;
	}
	if (!XU_WriteToASIC(sfReadWriteTriggerAddr, 0x1)){
		return FALSE;
	}
	if (!XU_SFWaitReady()){
		return FALSE;
	}
	if (!XU_WriteToASIC(sfCSAddr, 0x1)){
		return FALSE;
	}
	if (!XU_SFCMDreadStatus()){
		return FALSE;
	}
	if (!XU_WriteToASIC(sfModeAddr, 0x0)){
		return FALSE;
	}

    return TRUE;
}

BOOL XU_EraseSectorForSerialFlash(LONG addr, SERIAL_FLASH_TYPE sft)
{
		// disable serial falsh write protect
	BYTE sectorEraseCode = 0x20;
	switch (sft)
	{
	case SFT_ST:
		sectorEraseCode = 0xd8;
		break;
	case SFT_SST:
	case SFT_MXIC:
	case SFT_GIGA:
	case SFT_WINBOND:
	case SFT_MXIC_LIKE:
	case SFT_ATMEL_AT25F:
	case SFT_ATMEL_AT25FS:
	case SFT_ATMEL_AT45DB:
	case SFT_AMIC:
	case SFT_EON:
	case SFT_FENTECH:
	case SFT_UC:
	case SFT_BY:
	case SFT_FM:
	case SFT_ZB:
		sectorEraseCode = 0x20;
		break;
	case SFT_PMC:
		sectorEraseCode = 0xd7;
		break;
	case SFT_UNKNOW:
		return FALSE;	
		break;
	default:
		break;
	}

	BYTE data = 0x1;
	XU_WriteToASIC(sfModeAddr, data);//flash mode
	data = 0x0;
	XU_WriteToASIC(sfCSAddr, data);//CS=0
	data = 0x06;
	XU_WriteToASIC(sfWriteDataAddr, data);//WREN
	data = 0x01;
	XU_WriteToASIC(sfReadWriteTriggerAddr, data);
	XU_SFWaitReady();
	data = 0x1;
	XU_WriteToASIC(sfCSAddr, data);//CS=1
	//sector erase
	data = 0x0;
	XU_WriteToASIC(sfCSAddr, data);//CS=0
	data = sectorEraseCode;
	XU_WriteToASIC(sfWriteDataAddr, data);	// for chip sector erase
	//SetRegData(0x1082,0x20);
	data = 0x01;
	XU_WriteToASIC(sfReadWriteTriggerAddr, data);
	XU_SFWaitReady();
	//ldata = 0x00;
	data = addr >> 16;
	XU_WriteToASIC(sfWriteDataAddr, data);//addr
	//SetRegData(0x1082,0x00);
	data = 0x01;
	XU_WriteToASIC(sfReadWriteTriggerAddr, data);
	XU_SFWaitReady();
	data = addr >> 8;
	XU_WriteToASIC(sfWriteDataAddr, data);
	//SetRegData(0x1082,SectorNum);
	data = 0x01;
	XU_WriteToASIC(sfReadWriteTriggerAddr, data);
	XU_SFWaitReady();
	data = (BYTE)addr;//ldata = 0x00;
	XU_WriteToASIC(sfWriteDataAddr, data);
	//SetRegData(0x1082,0x00);
	data = 0x01;
	XU_WriteToASIC(sfReadWriteTriggerAddr, data);
	XU_SFWaitReady();
	data = 0x1;
	XU_WriteToASIC(sfCSAddr, data);//CS=1
	//SF_CMDread_Status
	XU_SFWaitReady();//SF_CMDread_Status(iDevIndex);
	data = 0x0;
	XU_WriteToASIC(sfModeAddr, data);//flash mode disable
	return TRUE;
}

BOOL XU_EraseBlockForSerialFlash(LONG addr, SERIAL_FLASH_TYPE sft)
{
// disable serial falsh write protect
	BYTE sectorEraseCode = 0x52;
	switch (sft)
	{
	case SFT_ST:
	case SFT_GIGA:
	case SFT_FENTECH:
	case SFT_UC:
	case SFT_BY:
		sectorEraseCode = 0xd8;
		break;
	case SFT_SST:
	case SFT_MXIC:
	case SFT_WINBOND:
	case SFT_MXIC_LIKE:
	case SFT_ATMEL_AT25F:
	case SFT_ATMEL_AT25FS:
	case SFT_ATMEL_AT45DB:
	case SFT_AMIC:
	case SFT_EON:
	case SFT_PMC:
	case SFT_FM:
	case SFT_ZB:
		sectorEraseCode = 0x52;
		break;
	case SFT_UNKNOW:
		return FALSE;
		break;
	default:
		break;
	}

	BYTE data = 0x1;
	XU_WriteToASIC(sfModeAddr, data);//flash mode
	data = 0x0;
	XU_WriteToASIC(sfCSAddr, data);//CS=0
	data = 0x06;
	XU_WriteToASIC(sfWriteDataAddr, data);//WREN
	data = 0x01;
	XU_WriteToASIC(sfReadWriteTriggerAddr, data);
	XU_SFWaitReady();
	data = 0x1;
	XU_WriteToASIC(sfCSAddr, data);//CS=1
	//sector erase
	data = 0x0;
	XU_WriteToASIC(sfCSAddr, data);//CS=0
	data = sectorEraseCode;
	XU_WriteToASIC(sfWriteDataAddr, data);	// for chip sector erase
	//SetRegData(0x1082,0x20);
	data = 0x01;
	XU_WriteToASIC(sfReadWriteTriggerAddr, data);
	XU_SFWaitReady();
	//ldata = 0x00;
	data = addr >> 16;
	XU_WriteToASIC(sfWriteDataAddr, data);//addr
	//SetRegData(0x1082,0x00);
	data = 0x01;
	XU_WriteToASIC(sfReadWriteTriggerAddr, data);
	XU_SFWaitReady();
	data = addr >> 8;
	XU_WriteToASIC(sfWriteDataAddr, data);
	//SetRegData(0x1082,SectorNum);
	data = 0x01;
	XU_WriteToASIC(sfReadWriteTriggerAddr, data);
	XU_SFWaitReady();
	data = (BYTE)addr;//ldata = 0x00;
	XU_WriteToASIC(sfWriteDataAddr, data);
	//SetRegData(0x1082,0x00);
	data = 0x01;
	XU_WriteToASIC(sfReadWriteTriggerAddr, data);
	XU_SFWaitReady();
	data = 0x1;
	XU_WriteToASIC(sfCSAddr, data);//CS=1
	//SF_CMDread_Status
	XU_SFWaitReady();//SF_CMDread_Status(iDevIndex);
	data = 0x0;
	XU_WriteToASIC(sfModeAddr, data);//flash mode disable
	return TRUE;
}

BOOL XU_SerialFlashErase(SERIAL_FLASH_TYPE sft)
{
	BOOL ret = TRUE;
	switch (sft){
	case SF_WINBOND:
	case SF_PMC:
	case SF_ST:
	case SF_AMIC:
		XU_WriteToASIC(sfModeAddr, 0x1);
		//SF_Set_WEL_Bit
		XU_WriteToASIC(sfCSAddr, 0x0);
		XU_WriteToASIC(sfWriteDataAddr, 0x06);
		XU_WriteToASIC(sfReadWriteTriggerAddr, 0x01);
		XU_SFWaitReady();
		XU_WriteToASIC(sfCSAddr, 0x1);
		//chip erase
		XU_WriteToASIC(sfCSAddr, 0x0);
		XU_WriteToASIC(sfWriteDataAddr, 0xc7);	// for PMC chip erase
		XU_WriteToASIC(sfReadWriteTriggerAddr, 0x01);
		XU_SFWaitReady();
		XU_WriteToASIC(sfCSAddr, 0x1);
		XU_SFCMDreadStatus();
		ret = XU_WriteToASIC(sfModeAddr, 0x0);
		break;
	case SF_SST:
		XU_WriteToASIC(sfModeAddr, 0x1);	// serial flash mode
		//SF_Set_EWSR_Bit
		XU_WriteToASIC(sfCSAddr, 0x0);	// chip select
		XU_WriteToASIC(sfWriteDataAddr, 0x50);	// write data
		XU_WriteToASIC(sfReadWriteTriggerAddr, 0x01);	// trigger for write
		XU_SFWaitReady();
		XU_WriteToASIC(sfCSAddr, 0x1);
		//SF_Set_WRSR_Bit
		XU_WriteToASIC(sfCSAddr, 0x0);	// chip select
		XU_WriteToASIC(sfWriteDataAddr, 0x01);	// write data
		XU_WriteToASIC(sfReadWriteTriggerAddr, 0x01);	// trigger for write
		XU_SFWaitReady();
		XU_WriteToASIC(sfWriteDataAddr, 0x00);
		XU_WriteToASIC(sfReadWriteTriggerAddr, 0x01);	// trigger for write
		XU_SFWaitReady();
		XU_WriteToASIC(sfCSAddr, 0x1);
		//SF_Set_WEL_Bit
		XU_WriteToASIC(sfCSAddr, 0x0);	// chip select
		XU_WriteToASIC(sfWriteDataAddr, 0x06);	// write data
		XU_WriteToASIC(sfReadWriteTriggerAddr, 0x01);	// trigger for write
		XU_SFWaitReady();
		XU_WriteToASIC(sfCSAddr, 0x1);
		//chip erase
		XU_WriteToASIC(sfCSAddr, 0x0);
		XU_WriteToASIC(sfWriteDataAddr, 0x60);
		XU_WriteToASIC(sfReadWriteTriggerAddr, 0x01);
		XU_SFWaitReady();
		XU_WriteToASIC(sfCSAddr, 0x1);
		//SF_CMDread_Status
		XU_SFCMDreadStatus();
		ret = XU_WriteToASIC(sfModeAddr, 0x0);
		break;
	case SF_UNKNOW:
	case SF_MXIC:
	case SF_ATMEL_AT25FS:
	case SF_MXIC_LIKE:
	case SF_FENTECH:
	case SF_UC:
	case SF_BY:
	case SF_FM:
	case SF_ZB:
	default:
		XU_WriteToASIC(sfModeAddr, 0x1);
		XU_WriteToASIC(sfCSAddr, 0x0);//CS#
		//SF_Set_WEL_Bit
		XU_WriteToASIC(sfWriteDataAddr, 0x06);//write enable cmd
		XU_WriteToASIC(sfReadWriteTriggerAddr, 0x01);
		XU_SFWaitReady();
		//disable BP0 BP1
		XU_WriteToASIC(sfCSAddr, 0x1);
		//chip erase
		XU_WriteToASIC(sfCSAddr, 0x0);
		XU_WriteToASIC(sfWriteDataAddr, 0x60);//chip erase cmd
		//SetRegData(0x1082,0xc7);
		XU_WriteToASIC(sfReadWriteTriggerAddr, 0x01);
		XU_SFWaitReady();
		XU_WriteToASIC(sfCSAddr, 0x1);
		//SF_CMDread_Status
		XU_SFCMDreadStatus();
		ret = XU_WriteToASIC(sfModeAddr, 0x0);
		break;
	}
    return TRUE;
}

BOOL XU_GetMemType(BYTE *pMemType)
{
	int ret = 0;
	__u8 ctrldata[11] = {0};

	//uvc_xu_control parmeters
	__u8 xu_unit= g_curExtendUnitID; 
	__u8 xu_selector= 5;
	__u16 xu_size= 11;
	__u8 *xu_data= ctrldata;

	if ((ret=XU_Get_Cur(xu_unit, xu_selector, xu_size, xu_data)) < 0) 
	{
		printf("ioctl(UVCIOC_CTRL_GET) FAILED (%i)\n",ret);
		//if(ret==EINVAL)			printf("Invalid arguments\n");
		return FALSE;
	}

	*pMemType = xu_data[2];
    return TRUE;
}

BOOL XU_ReadBitFormAsic(LONG addr, BYTE bit)
{
	BYTE bufs;
	BYTE data;
	XU_ReadFromASIC(addr, &bufs);

	switch (bit)
	{
	case 0:
		data = bufs & 0x01;
		break;
	case 1:
		data = bufs & 0x02;
		break;
	case 2:
		data = bufs & 0x04;
		break;
	case 3:
		data = bufs & 0x08;
		break;
	case 4:
		data = bufs & 0x10;
		break;
	case 5:
		data = bufs & 0x20;
		break;
	case 6:
		data = bufs & 0x40;
		break;
	case 7:
		data = bufs & 0x80;
		break;
	default:
		break;
	}
	return data;
}

BOOL XU_SFWaitReady()
{
	BYTE i;
	LONG data = 0;
	for (i = 0; i < 50; i++)										//pooling ready; 500us timeout
	{
		if (XU_ReadBitFormAsic(sfRdyAddr, 0))                      //aISP_RDY
		{
			return TRUE;
		}
		usleep(1000);
	}
	return FALSE;
}

BOOL XU_SFCMDreadStatus()
{
	int i;
	unsigned char ucData;
	for (i = 0; i < 10000; ++i){
		// chip select to low
		if (TRUE != XU_WriteToASIC(sfCSAddr, 0x0)){
			return FALSE;
		}
		// Read status cmd
		if (TRUE != XU_WriteToASIC(sfWriteDataAddr, 0x05)){
			return FALSE;
		}
		// Write trig
		if (TRUE != XU_WriteToASIC(sfReadWriteTriggerAddr, 0x01)){
			return FALSE;
		}
		if (TRUE != XU_SFWaitReady()){
			return FALSE;
		}
		// Read reg cmd 
		if (TRUE != XU_WriteToASIC(sfReadDataAddr, 0x0)){
			return FALSE;;
		}
		// 2: Trigger for read data to Serial Flash in SF_MODE
		if (TRUE != XU_WriteToASIC(sfReadWriteTriggerAddr, 0x02)){
			return FALSE;
		}
		if (TRUE != XU_SFWaitReady()){
			return FALSE;
		}
		// Data read from Serial Flash
		if (TRUE != XU_ReadFromASIC(sfReadDataAddr, &ucData)){
			return FALSE;
		}
		// chip select to high
		if ((ucData & 0x01) != 0x01){
			if (TRUE != XU_WriteToASIC(sfCSAddr, 0x1)){
				return FALSE;
			}
			return TRUE;
		}
		usleep(1000);
	}
	return FALSE;
}

BOOL XU_GetParaTableAndCRCAddrFormFW(BYTE *pFW, ULONG* paraTableStartAddr, ULONG* paraTableEndAddr, ULONG* crcAddr)
{
	BYTE romVersion[8] = { 0 };
	if (!XU_GetAsicRomVersion(romVersion))
		return FALSE;

	if (memcmp(romVersion, "231R0", 4) == 0 && romVersion[5] >= 2) // 231R0_V2
	{
		return FALSE;
	}
	else if (((memcmp(romVersion, "232R0", 4) == 0 && romVersion[5] == 1)) ||
		((memcmp(romVersion, "275R0", 4) == 0 && romVersion[5] == 1 && romVersion[6] == 0x30)) ||
		//((memcmp(romVersion, "290R0", 4) == 0 && romVersion[5] == 1)) ||
		((memcmp(romVersion, "276R0", 4) == 0 && romVersion[5] == 1)))
	{
		*paraTableStartAddr = 0xC000;
		*paraTableEndAddr = *paraTableStartAddr + 0xF00;
		*crcAddr = *paraTableStartAddr + 0xF00;
	}
	else if (((memcmp(romVersion, "232R0", 4) == 0 && romVersion[5] == 2)) ||
		//((memcmp(romVersion, "290R0", 4) == 0 && romVersion[5] == 2)) ||
		((memcmp(romVersion, "288R0", 4) == 0 && romVersion[5] == 1)) ||
		//((memcmp(romVersion, "289R0", 4) == 0 && romVersion[5] == 1)) ||
		((memcmp(romVersion, "270R0", 4) == 0 && romVersion[5] == 1)) ||
		((memcmp(romVersion, "271R0", 4) == 0 && romVersion[5] == 1)) ||
		((memcmp(romVersion, "280R0", 4) == 0 && romVersion[5] == 1)) ||
		((memcmp(romVersion, "281R0", 4) == 0 && romVersion[5] == 1)))
	{
		BYTE sectorTable[32];
		memcpy(sectorTable, pFW + 0x160, sizeof(sectorTable));

		// pbySectorTable[8] is parameter start address for deivce
		// pbySectorTable[9] is parameter size for deivce
		*paraTableStartAddr = (ULONG)sectorTable[0x08] << 8;
		*paraTableEndAddr = *paraTableStartAddr + ((ULONG)sectorTable[0x09] << 8);
		*crcAddr = (ULONG)sectorTable[0xE] << 8;
	}
	else if (((memcmp(romVersion, "272R0", 4) == 0) && (romVersion[5] == 1)) ||
		((memcmp(romVersion, "273R0", 4) == 0) && (romVersion[5] == 1)) ||
		((memcmp(romVersion, "275R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
		((memcmp(romVersion, "283R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
		((memcmp(romVersion, "285R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
		((memcmp(romVersion, "286R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
		((memcmp(romVersion, "289R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) || 
		((memcmp(romVersion, "290R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) || 
		((memcmp(romVersion, "287R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
		((memcmp(romVersion, "267R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
		((memcmp(romVersion, "292R0", 4) == 0) && (romVersion[5] == 1)))//wei add 292
	{
		// Get Parameter table start address
		// Parameter table start address is stored at 0x16F 
		BYTE sectorTable[0x2B];
		memcpy(sectorTable, pFW + 0x160, sizeof(sectorTable));

		*paraTableStartAddr = ((ULONG)sectorTable[0x0F] << 24) + ((ULONG)sectorTable[0x10] << 16) + ((ULONG)sectorTable[0x11] << 8) + sectorTable[0x12];
		ULONG dwParaTableSize = ((ULONG)sectorTable[0x13] << 24) + ((ULONG)sectorTable[0x14] << 16) + ((ULONG)sectorTable[0x15] << 8) + sectorTable[0x16];
		*paraTableEndAddr = *paraTableStartAddr + dwParaTableSize;
		*crcAddr = ((ULONG)sectorTable[0x27] << 24) + ((ULONG)sectorTable[0x28] << 16) + ((ULONG)sectorTable[0x29] << 8) + sectorTable[0x2a];
	}
	else if ((memcmp(romVersion, "216R0", 4) == 0))
	{
		*paraTableStartAddr = 0x5800;
		*paraTableEndAddr = *paraTableStartAddr + 0xF00;
		*crcAddr = *paraTableStartAddr + 0xF00;
	}
	else
	{
		*paraTableStartAddr = 0x8000;
		*paraTableEndAddr = *paraTableStartAddr + 0x800;
		*crcAddr = 0;
	}
	return TRUE;
}

BOOL XU_GetParaTableAndCRCAddrFormSF(ULONG *paraTableStartAddr, ULONG *paraTableEndAddr, ULONG *crcAddr)
{
	BYTE romVersion[8] = { 0 };
	if (TRUE != XU_GetAsicRomVersion(romVersion))
		return FALSE;

	if (memcmp(romVersion, "231R0", 4) == 0 && romVersion[5] >= 2) // 231R0_V2
	{
		return FALSE;
	}
	else if (((memcmp(romVersion, "232R0", 4) == 0 && romVersion[5] == 1)) ||
		((memcmp(romVersion, "275R0", 4) == 0 && romVersion[5] == 1 && romVersion[6] == 0x30)) ||
		//((memcmp(romVersion, "290R0", 4) == 0 && romVersion[5] == 1)) ||
		((memcmp(romVersion, "276R0", 4) == 0 && romVersion[5] == 1)) )
	{
		*paraTableStartAddr = 0xC000;
		*paraTableEndAddr = *paraTableStartAddr + 0xF00;
		*crcAddr = *paraTableStartAddr + 0xF00;
	}
	else if (((memcmp(romVersion, "232R0", 4) == 0 && romVersion[5] == 2)) ||
		//((memcmp(romVersion, "290R0", 4) == 0 && romVersion[5] == 2)) ||
		((memcmp(romVersion, "288R0", 4) == 0 && romVersion[5] == 1)) ||
		//((memcmp(romVersion, "289R0", 4) == 0 && romVersion[5] == 1)) ||
		((memcmp(romVersion, "270R0", 4) == 0 && romVersion[5] == 1)) ||
		((memcmp(romVersion, "271R0", 4) == 0 && romVersion[5] == 1)) ||
		((memcmp(romVersion, "280R0", 4) == 0 && romVersion[5] == 1)) ||
		((memcmp(romVersion, "281R0", 4) == 0 && romVersion[5] == 1)))
	{
		BYTE sectorTable[32];
		if (TRUE != XU_ReadFormSF(0x160, sectorTable, sizeof(sectorTable)))
			return FALSE;

		// pbySectorTable[8] is parameter start address for deivce
		// pbySectorTable[9] is parameter size for deivce
		*paraTableStartAddr = (ULONG)sectorTable[0x08] << 8;
		*paraTableEndAddr = *paraTableStartAddr + ((ULONG)sectorTable[0x09] << 8);
		*crcAddr = (ULONG)sectorTable[0xE] << 8;
	}
	else if (((memcmp(romVersion, "272R0", 4) == 0) && (romVersion[5] == 1)) ||
		((memcmp(romVersion, "273R0", 4) == 0) && (romVersion[5] == 1)) ||
		((memcmp(romVersion, "275R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
		((memcmp(romVersion, "283R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
		((memcmp(romVersion, "285R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
		((memcmp(romVersion, "286R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
		((memcmp(romVersion, "289R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) || 
		((memcmp(romVersion, "290R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) || 
		((memcmp(romVersion, "287R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
		((memcmp(romVersion, "267R0", 4) == 0) && (romVersion[5] == 1) && (romVersion[6] == 0x46)) ||
		((memcmp(romVersion, "292R0", 4) == 0) && (romVersion[5] == 1)))//wei add 292
	{
		// Get Parameter table start address
		// Parameter table start address is stored at 0x16F
		BYTE sectorTable[0x2B];
		if (TRUE != XU_ReadFormSF(0x160, sectorTable, sizeof(sectorTable)))
			return FALSE;

		*paraTableStartAddr = ((ULONG)sectorTable[0x0F] << 24) + ((ULONG)sectorTable[0x10] << 16) + ((ULONG)sectorTable[0x11] << 8) + sectorTable[0x12];
		ULONG dwParaTableSize = ((ULONG)sectorTable[0x13] << 24) + ((ULONG)sectorTable[0x14] << 16) + ((ULONG)sectorTable[0x15] << 8) + sectorTable[0x16];
		*paraTableEndAddr = *paraTableStartAddr + dwParaTableSize;
		*crcAddr = ((ULONG)sectorTable[0x27] << 24) + ((ULONG)sectorTable[0x28] << 16) + ((ULONG)sectorTable[0x29] << 8) + sectorTable[0x2a];
	}
	else if ((memcmp(romVersion, "216R0", 4) == 0))
	{
		*paraTableStartAddr = 0x5800;
		*paraTableEndAddr = *paraTableStartAddr + 0xF00;
		*crcAddr = *paraTableStartAddr + 0xF00;
	}
	else
	{
		*paraTableStartAddr = 0x8000;
		*paraTableEndAddr = *paraTableStartAddr + 0x800;
		*crcAddr = 0;
	}
    return TRUE;
}

BOOL XU_GetStringSettingFormSF(BYTE* pbyString, DWORD stringSize, DWORD StringOffset, BOOL bIsCRCProtect)
{
	DWORD dwStringAddr = 0;
	ULONG dwParaTableStartAddr = 0;
	ULONG dwParaTableEndAddr = 0;
	ULONG dwCRCStartAddr = 0;

	if (TRUE != XU_GetParaTableAndCRCAddrFormSF(&dwParaTableStartAddr, &dwParaTableEndAddr, &dwCRCStartAddr))
		return FALSE;

	dwStringAddr = StringOffset;
	if (bIsCRCProtect)
		dwStringAddr += dwParaTableStartAddr;
	else
		dwStringAddr += dwCRCStartAddr;

	BYTE pbyStringBuf[0x40] = { 0 };
	if (TRUE != XU_ReadFormSF(dwStringAddr, pbyStringBuf, sizeof(pbyStringBuf)))
		return FALSE;

	// Calculate string length
	DWORD dwStringLength = 0;
	if (bIsCRCProtect)
		dwStringLength = (pbyStringBuf[0] - 2) / 2;
	else
	for (; (dwStringLength < 0x40 / 2) && (pbyStringBuf[dwStringLength] != 0xFF); ++dwStringLength);

	if (stringSize < dwStringLength)
		return FALSE;

	// Copy string to output buffer
	DWORD i;
	if (bIsCRCProtect)
	{
		for (i = 0; i<dwStringLength; ++i)
			pbyString[i] = pbyStringBuf[2 + i * 2];
	}
	else
	{
		memcpy(pbyString, pbyStringBuf, dwStringLength);
	}

    return TRUE;
}
