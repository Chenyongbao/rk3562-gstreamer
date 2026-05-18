#include "usbcam_toolkit.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include "SonixCamera.h"
#include "util.h"

static SERIAL_FLASH_TYPE get_flash_type_or_fallback(void);

static int read_entire_file_u8(const char* path, uint8_t** out_buf, size_t* out_len)
{
    *out_buf = NULL;
    *out_len = 0;

    FILE* fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "fopen(%s) failed: %s\n", path, strerror(errno));
        return 1;
    }
    if (fseek(fp, 0, SEEK_END) != 0) {
        fprintf(stderr, "fseek failed: %s\n", strerror(errno));
        fclose(fp);
        return 1;
    }
    long sz = ftell(fp);
    if (sz < 0) {
        fprintf(stderr, "ftell failed: %s\n", strerror(errno));
        fclose(fp);
        return 1;
    }
    if (fseek(fp, 0, SEEK_SET) != 0) {
        fprintf(stderr, "fseek failed: %s\n", strerror(errno));
        fclose(fp);
        return 1;
    }

    uint8_t* buf = (uint8_t*)malloc((size_t)sz);
    if (!buf) {
        fprintf(stderr, "malloc(%ld) failed\n", sz);
        fclose(fp);
        return 1;
    }
    size_t n = fread(buf, 1, (size_t)sz, fp);
    fclose(fp);
    if (n != (size_t)sz) {
        fprintf(stderr, "fread failed: got %zu/%ld\n", n, sz);
        free(buf);
        return 1;
    }

    *out_buf = buf;
    *out_len = n;
    return 0;
}

static int v4l2_open_video(int videoIndex)
{
    char devPath[64];
    snprintf(devPath, sizeof(devPath), "/dev/video%d", videoIndex);
    int fd = open(devPath, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "open(%s) failed: %s\n", devPath, strerror(errno));
        return -1;
    }
    return fd;
}

static int v4l2_gain_query(int fd, struct v4l2_queryctrl* out_q)
{
    memset(out_q, 0, sizeof(*out_q));
    out_q->id = V4L2_CID_GAIN;
    if (ioctl(fd, VIDIOC_QUERYCTRL, out_q) < 0) {
        if (errno == EINVAL) {
            fprintf(stderr, "V4L2_CID_GAIN not supported by this device\n");
        } else {
            fprintf(stderr, "VIDIOC_QUERYCTRL(V4L2_CID_GAIN) failed: %s\n", strerror(errno));
        }
        return 1;
    }
    if (out_q->flags & V4L2_CTRL_FLAG_DISABLED) {
        fprintf(stderr, "V4L2_CID_GAIN is disabled on this device\n");
        return 2;
    }
    if (out_q->type != V4L2_CTRL_TYPE_INTEGER && out_q->type != V4L2_CTRL_TYPE_INTEGER64) {
        fprintf(stderr, "V4L2_CID_GAIN has unsupported type=%d\n", (int)out_q->type);
        return 3;
    }
    return 0;
}

int usbcamGetLightStatus(int videoIndex, int* status)
{
    if (!status) return 1;
    int fd = v4l2_open_video(videoIndex);
    if (fd < 0) return 2;

    struct v4l2_queryctrl q;
    if (v4l2_gain_query(fd, &q) != 0) {
        close(fd);
        return 3;
    }

    struct v4l2_control c;
    memset(&c, 0, sizeof(c));
    c.id = V4L2_CID_GAIN;
    if (ioctl(fd, VIDIOC_G_CTRL, &c) < 0) {
        fprintf(stderr, "VIDIOC_G_CTRL(V4L2_CID_GAIN) failed: %s\n", strerror(errno));
        close(fd);
        return 4;
    }
    *status = c.value;
    close(fd);
    return 0;
}

int usbcamSetLightStatus(int videoIndex, int brightness)
{
    int fd = v4l2_open_video(videoIndex);
    if (fd < 0) return 2;

    struct v4l2_queryctrl q;
    if (v4l2_gain_query(fd, &q) != 0) {
        close(fd);
        return 3;
    }
    if (brightness < q.minimum || brightness > q.maximum) {
        fprintf(stderr, "GAIN value out of range: %d (min=%d max=%d)\n", brightness, q.minimum, q.maximum);
        close(fd);
        return 5;
    }

    struct v4l2_control c;
    memset(&c, 0, sizeof(c));
    c.id = V4L2_CID_GAIN;
    c.value = brightness;
    if (ioctl(fd, VIDIOC_S_CTRL, &c) < 0) {
        fprintf(stderr, "VIDIOC_S_CTRL(V4L2_CID_GAIN=%d) failed: %s\n", brightness, strerror(errno));
        close(fd);
        return 6;
    }
    close(fd);
    return 0;
}

uint8_t usbcamCalib_calc_checksum(const CalibData* data)
{
    return calibdata_calc_checksum(data);
}

static int verify_calib_checksum(const CalibData* d)
{
    if (!d) return 1;
    uint8_t calc = usbcamCalib_calc_checksum(d);
    return (d->checksum == calc) ? 0 : 2;
}

int usbcamCalibReadFlash(int videoIndex, long addr, CalibData* out)
{
    if (!out) return 1;
    if (!SonixCam_Init(videoIndex)) {
        fprintf(stderr, "SonixCam_Init failed for /dev/video%d\n", videoIndex);
        return 2;
    }

    uint8_t buf[sizeof(CalibData)];
    if (!SonixCam_SerialFlashCustomRead(addr, buf, (long)sizeof(buf))) {
        fprintf(stderr, "SerialFlashCustomRead failed at 0x%05lX\n", addr);
        SonixCam_UnInit();
        return 3;
    }
    memcpy(out, buf, sizeof(*out));

    int rc = verify_calib_checksum(out);
    if (rc != 0) {
        uint8_t calc = usbcamCalib_calc_checksum(out);
        fprintf(stderr, "CalibData checksum mismatch: read=0x%02X calc=0x%02X\n",
                (unsigned)out->checksum, (unsigned)calc);
        SonixCam_UnInit();
        return 4;
    }

    SonixCam_UnInit();
    return 0;
}

int usbcamCalibWriteFlash(int videoIndex, long addr, CalibData data)
{
    data.checksum = usbcamCalib_calc_checksum(&data);

    if (!SonixCam_Init(videoIndex)) {
        fprintf(stderr, "SonixCam_Init failed for /dev/video%d\n", videoIndex);
        return 2;
    }

#if 1

    //旧实现：先整块读取扇区、写回再整块校验，保证其它数据不被破坏。

    uint8_t sector[SONIX_CALIB_SECTOR_SIZE];
    if (!SonixCam_SerialFlashCustomRead(addr, sector, (long)sizeof(sector))) {
        fprintf(stderr, "SerialFlashCustomRead failed at 0x%05lX\n", addr);
        SonixCam_UnInit();
        return 3;
    }
    memcpy(sector, &data, sizeof(data));
    SERIAL_FLASH_TYPE sft = SFT_WINBOND;

    if (!SonixCam_SerialFlashSectorCustomWrite(addr, sector, (long)sizeof(sector), sft)) {
        fprintf(stderr, "SerialFlashSectorCustomWrite failed at 0x%05lX\n", addr);
        SonixCam_UnInit();
        return 4;
    }
    uint8_t verify[SONIX_CALIB_SECTOR_SIZE];
    if (!SonixCam_SerialFlashCustomRead(addr, verify, (long)sizeof(verify))) {
        fprintf(stderr, "SerialFlashCustomRead verify failed at 0x%05lX\n", addr);
        SonixCam_UnInit();
        return 5;
    }
    if (memcmp(sector, verify, sizeof(sector)) != 0) {
        fprintf(stderr, "Verify mismatch at 0x%05lX\n", addr);
        SonixCam_UnInit();
        return 6;
    }
#endif

#if 0

    if (!SonixCam_SerialFlashWrite(addr, (unsigned char*)&data, (long)sizeof(data))) {
        fprintf(stderr, "SerialFlashWrite failed at 0x%05lX\n", addr);
        SonixCam_UnInit();
        return 3;
    }
#endif

    SonixCam_UnInit();
    return 0;
}

static SERIAL_FLASH_TYPE get_flash_type_or_fallback(void)
{
    SERIAL_FLASH_TYPE sft = SFT_UNKNOW;
    if (!SonixCam_GetSerialFlashType(&sft, true) || sft == SFT_UNKNOW) {
        return SFT_WINBOND;
    }
    return sft;
}

static int check_range_erased(long start_addr, long length)
{
    if (length <= 0) return 0;
    const long step = 4096L;
    uint8_t buf[64];
    long end_addr = start_addr + length;

    for (long addr = start_addr; addr < end_addr; addr += step) {
        int ok = 0;
        for (int attempt = 0; attempt < 30; ++attempt) {
            if (!SonixCam_SerialFlashCustomRead(addr, buf, (long)sizeof(buf))) {
                fprintf(stderr, "Erase verify read failed at 0x%05lX\n", addr);
                return 2;
            }
            ok = 1;
            for (size_t i = 0; i < sizeof(buf); ++i) {
                if (buf[i] != 0xFF) { ok = 0; break; }
            }
            if (ok) break;
            usleep(100000);
        }
        if (!ok) {
            fprintf(stderr, "Erase verify mismatch at 0x%05lX (e.g. first=0x%02X)\n",
                    addr, (unsigned)buf[0]);
            return 3;
        }
    }
    return 0;
}

static int erase_range_blocks_32k(long start_addr, long length, SERIAL_FLASH_TYPE sft)
{
    const long block_size = 0x8000L; // 32KB
    if (length <= 0) return 0;
    long end_addr = start_addr + length;
    long addr = (start_addr / block_size) * block_size;
    if (addr < 0) addr = 0;
    for (; addr < end_addr; addr += block_size) {
        if (!SonixCam_EraseBlockFlash(addr, sft)) {
            fprintf(stderr, "EraseBlockFlash(32KB) failed at 0x%05lX\n", addr);
            return 1;
        }
    }
    return 0;
}

static int verify_calib_at(long addr)
{
    uint8_t buf[sizeof(CalibData)];
    if (!SonixCam_SerialFlashCustomRead(addr, buf, (long)sizeof(buf))) {
        return 1;
    }
    CalibData d;
    memcpy(&d, buf, sizeof(d));
    return verify_calib_checksum(&d);
}

int usbcamUpgradeFwPreserveCalib(int videoIndex, const char* fw_path, long calib_addr)
{
    if (!fw_path) return 2;
    if (!SonixCam_Init(videoIndex)) {
        fprintf(stderr, "SonixCam_Init failed for /dev/video%d\n", videoIndex);
        return 1;
    }

    uint8_t calib_sector_backup[SONIX_CALIB_SECTOR_SIZE];
    if (!SonixCam_SerialFlashCustomRead(calib_addr, calib_sector_backup, SONIX_CALIB_SECTOR_SIZE)) {
        fprintf(stderr, "Backup failed @0x%05lX\n", calib_addr);
        SonixCam_UnInit();
        return 3;
    }

    int pre = verify_calib_at(calib_addr);
    if (pre == 0) printf("Pre-upgrade CalibData checksum OK\n");
    else fprintf(stderr, "Warning: pre-upgrade CalibData checksum not OK (rc=%d)\n", pre);

    uint8_t* fw = NULL;
    size_t fw_len = 0;
    if (read_entire_file_u8(fw_path, &fw, &fw_len) != 0) {
        SonixCam_UnInit();
        return 4;
    }
    printf("FW file: %s (%zu bytes)\n", fw_path, fw_len);

    SERIAL_FLASH_TYPE sft = get_flash_type_or_fallback();
    if (sft == SFT_WINBOND) printf("Flash type unknown; fallback to SFT_WINBOND\n");
    else printf("Flash type: %d\n", (int)sft);

    printf("Start firmware burn...\n");
    if (!SonixCam_BurnerFW(fw, (LONG)fw_len, NULL, NULL, sft, FALSE)) {
        fprintf(stderr, "SonixCam_BurnerFW failed\n");
        free(fw);
        SonixCam_UnInit();
        return 5;
    }
    printf("Firmware burn OK\n");
    free(fw);

    printf("Restore Calib sector @0x%05lX...\n", calib_addr);
    if (!SonixCam_SerialFlashSectorCustomWrite(calib_addr, calib_sector_backup, SONIX_CALIB_SECTOR_SIZE, sft)) {
        fprintf(stderr, "Restore failed @0x%05lX\n", calib_addr);
        SonixCam_UnInit();
        return 6;
    }

    int post = verify_calib_at(calib_addr);
    if (post != 0) {
        fprintf(stderr, "Post-upgrade CalibData checksum NOT OK (rc=%d)\n", post);
        SonixCam_UnInit();
        return 7;
    }
    printf("Post-upgrade CalibData checksum OK\n");

    SonixCam_UnInit();
    return 0;
}

int usbcamUpgradeFwPartial(int videoIndex, const char* fw_path, long calib_addr)
{
    if (!fw_path) return 2;
    if (!SonixCam_Init(videoIndex)) {
        fprintf(stderr, "SonixCam_Init failed for /dev/video%d\n", videoIndex);
        return 1;
    }

    uint8_t* fw = NULL;
    size_t fw_len = 0;
    if (read_entire_file_u8(fw_path, &fw, &fw_len) != 0) {
        SonixCam_UnInit();
        return 3;
    }
    printf("FW file: %s (%zu bytes)\n", fw_path, fw_len);

    const long fw_start = 0x00000L;
    const long fw_end = fw_start + (long)fw_len;
    const bool overlap_calib_sector = (fw_end > calib_addr) && (fw_start < (calib_addr + SONIX_CALIB_SECTOR_SIZE));

    uint8_t calib_sector_backup[SONIX_CALIB_SECTOR_SIZE];
    if (overlap_calib_sector) {
        if (!SonixCam_SerialFlashCustomRead(calib_addr, calib_sector_backup, SONIX_CALIB_SECTOR_SIZE)) {
            fprintf(stderr, "Backup failed @0x%05lX\n", calib_addr);
            free(fw);
            SonixCam_UnInit();
            return 4;
        }
        printf("Backup Calib sector @0x%05lX (overlap)\n", calib_addr);
    }

    SERIAL_FLASH_TYPE sft = get_flash_type_or_fallback();
    if (sft == SFT_WINBOND) printf("Flash type unknown; fallback to SFT_WINBOND\n");
    else printf("Flash type: %d\n", (int)sft);

    if (!SonixCam_DisableSerialFlashWriteProtect(sft)) {
        fprintf(stderr, "DisableSerialFlashWriteProtect failed\n");
        free(fw);
        SonixCam_UnInit();
        return 5;
    }

    printf("Erase range: [0x%05lX, 0x%05lX) (%zu bytes)\n", fw_start, fw_end, fw_len);
    if (erase_range_blocks_32k(fw_start, (long)fw_len, sft) != 0) {
        free(fw);
        SonixCam_UnInit();
        return 6;
    }
    int erase_ck = check_range_erased(fw_start, (long)fw_len);
    if (erase_ck != 0) {
        fprintf(stderr, "Erase verify failed (rc=%d); abort write\n", erase_ck);
        free(fw);
        SonixCam_UnInit();
        return 6;
    }

    printf("Start firmware write...\n");
    if (!SonixCam_WriteFwToFlash(fw, (LONG)fw_len, NULL, NULL, FALSE)) {
        fprintf(stderr, "SonixCam_WriteFwToFlash failed\n");
        free(fw);
        SonixCam_UnInit();
        return 7;
    }
    printf("Firmware write OK\n");
    free(fw);

    if (overlap_calib_sector) {
        printf("Restore Calib sector @0x%05lX...\n", calib_addr);
        if (!SonixCam_SerialFlashSectorCustomWrite(calib_addr, calib_sector_backup, SONIX_CALIB_SECTOR_SIZE, sft)) {
            fprintf(stderr, "Restore failed @0x%05lX\n", calib_addr);
            SonixCam_UnInit();
            return 8;
        }
    }

    int post = verify_calib_at(calib_addr);
    if (post == 0) printf("CalibData checksum OK\n");
    else fprintf(stderr, "CalibData checksum NOT OK (rc=%d)\n", post);

    SonixCam_UnInit();
    return 0;
}

