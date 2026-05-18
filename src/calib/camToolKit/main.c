#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <errno.h>

#include "usbcam_toolkit.h"

static void print_usage(const char* prog)
{
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "  %s <video_index> light get          - Get current PWM light brightness (0-100%%)\n", prog);
    fprintf(stderr, "  %s <video_index> light on           - Turn PWM light on (set to 100%%)\n", prog);
    fprintf(stderr, "  %s <video_index> light off          - Turn PWM light off (set to 0%%)\n", prog);
    fprintf(stderr, "  %s <video_index> light set <value> - Set PWM light brightness (0-100%%)\n", prog);
    fprintf(stderr, "  %s <video_index> calib read\n", prog);
    fprintf(stderr,
            "  %s <video_index> calib write <lens_type> fx fy cx cy k0 k1 k2 k3 k4\n",
            prog);
    fprintf(stderr,
            "  %s <video_index> calib write_yaml [yaml_path]\n",
            prog);
    fprintf(stderr,
            "                                     (default: ../calib1.yml if path not provided)\n");
    fprintf(stderr, "  %s <video_index> fw upgrade <fw_path> [--preserve-calib|--partial]\n", prog);
    fprintf(stderr, "                                     --preserve-calib: Full erase + preserve calib (default)\n");
    fprintf(stderr, "                                     --partial: Partial erase (only firmware area)\n");
    fprintf(stderr, "    lens_type: pinhole | fisheye\n");
    fprintf(stderr, "\nNote: PWM light control uses V4L2_CID_GAIN interface (manufacturer confirmed).\n");
    fprintf(stderr, "      Value 0%% = off, 1-100%% = brightness level.\n");
    fprintf(stderr, "      The actual hardware range is automatically mapped from 0-100%%.\n");
}

static int v4l2_query_gain_range(int video_index, int* min_val, int* max_val)
{
    char devPath[64];
    snprintf(devPath, sizeof(devPath), "/dev/video%d", video_index);
    int fd = open(devPath, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }
    
    struct v4l2_queryctrl q;
    memset(&q, 0, sizeof(q));
    q.id = V4L2_CID_GAIN;
    int ret = 0;
    if (ioctl(fd, VIDIOC_QUERYCTRL, &q) < 0) {
        ret = -1;
    } else {
        *min_val = q.minimum;
        *max_val = q.maximum;
    }
    close(fd);
    return ret;
}

static int cmd_light_get(int video_index)
{
    int status = 0;
    int rc = usbcamGetLightStatus(video_index, &status);
    if (rc != 0) {
        fprintf(stderr,
                "usbcamGetLightStatus failed for /dev/video%d (rc=%d)\n",
                video_index,
                rc);
        return 1;
    }
    
    // 查询设备支持的范围以计算百分比
    int min_val = 0, max_val = 100;
    int percentage = 0;
    if (v4l2_query_gain_range(video_index, &min_val, &max_val) == 0) {
        if (max_val > min_val) {
            percentage = ((status - min_val) * 100) / (max_val - min_val);
        } else {
            percentage = (status == min_val) ? 0 : 100;
        }
    } else {
        // 如果查询失败，假设范围是0-100
        percentage = status;
    }
    
    const char* state = (status == min_val) ? "OFF" : "ON";
    printf("/dev/video%d PWM light: %s (brightness=%d%%, actual=%d, range=[%d-%d])\n", 
           video_index, state, percentage, status, min_val, max_val);
    return 0;
}

/*
 * Minimal parser for OpenCV-style camera calibration YAML:
 *
 * image_width: 1920
 * image_height: 1080
 * camera_matrix: !!opencv-matrix
 *   rows: 3
 *   cols: 3
 *   dt: d
 *   data: [ fx, 0, cx, 0, fy, cy, 0, 0, 1 ]
 * distortion_coefficients: !!opencv-matrix
 *   rows: 1
 *   cols: 5
 *   dt: d
 *   data: [ k1, k2, p1, p2, k3 ]
 */

static int parse_opencv_matrix_data(FILE* fp,
                                    const char* first_line,
                                    int expected_count,
                                    double* out)
{
    char buf[1024];
    int count = 0;
    int found_bracket = 0;

    /* process the first line that contains "data:" */
    strncpy(buf, first_line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    while (1) {
        char* p = strchr(buf, '[');
        if (p) {
            found_bracket = 1;
            p++; // Skip the '['
        } else if (found_bracket) {
            // Already found bracket, continue from start of line
            p = buf;
        } else {
            // Haven't found bracket yet, try next line
            if (!fgets(buf, sizeof(buf), fp)) {
                break;
            }
            continue;
        }

        char* end = p;
        // Skip whitespace
        while (*end && (*end == ' ' || *end == '\t')) {
            end++;
        }

        while (*end && *end != ']') {
            double val;
            char* next;
            val = strtod(end, &next);
            if (next == end) {
                // No number found, skip this character
                if (*end == ',' || *end == ' ' || *end == '\t' || *end == '\n' || *end == '\r') {
                    end++;
                } else {
                    break;
                }
            } else {
                out[count++] = val;
                end = next;
                if (count >= expected_count) {
                    break;
                }
                // Skip whitespace and commas
                while (*end && (*end == ' ' || *end == '\t' || *end == ',' || *end == '\n' || *end == '\r')) {
                    end++;
                }
            }
        }

        if (count >= expected_count) {
            break;
        }

        if (*end == ']') {
            // Found closing bracket, we're done
            break;
        }

        if (!fgets(buf, sizeof(buf), fp)) {
            break;
        }
    }

    if (count != expected_count) {
        fprintf(stderr,
                "Expected %d values in matrix data but got %d\n",
                expected_count,
                count);
        return -1;
    }

    return 0;
}

static int find_and_parse_matrix(FILE* fp,
                                 const char* name,
                                 int expected_count,
                                 double* out)
{
    char line[512];
    int in_section = 0;

    while (fgets(line, sizeof(line), fp)) {
        if (!in_section) {
            if (strstr(line, name) != NULL) {
                in_section = 1;
            }
        } else {
            if (strstr(line, "data:") != NULL) {
                if (parse_opencv_matrix_data(fp, line, expected_count, out) != 0) {
                    return -1;
                }
                return 0;
            }
        }
    }

    fprintf(stderr, "Failed to find %s data in YAML file\n", name);
    return -1;
}

static int parse_opencv_yaml(const char* path,
                             double* fx,
                             double* fy,
                             double* cx,
                             double* cy,
                             double distCoeffs[5])
{
    FILE* fp = fopen(path, "r");
    if (!fp) {
        fprintf(stderr, "Failed to open YAML file: %s\n", path);
        return -1;
    }

    double cam[9];
    if (find_and_parse_matrix(fp, "camera_matrix", 9, cam) != 0) {
        fclose(fp);
        return -1;
    }

    if (find_and_parse_matrix(fp, "distortion_coefficients", 5, distCoeffs) != 0) {
        fclose(fp);
        return -1;
    }

    fclose(fp);

    /* OpenCV camera_matrix layout: [fx, 0, cx, 0, fy, cy, 0, 0, 1] */
    *fx = cam[0];
    *cx = cam[2];
    *fy = cam[4];
    *cy = cam[5];

    return 0;
}

static int cmd_calib_write_yaml(int video_index, const char* yaml_path)
{
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    double k[5] = {0};

    if (parse_opencv_yaml(yaml_path, &fx, &fy, &cx, &cy, k) != 0) {
        fprintf(stderr, "Failed to parse calibration YAML: %s\n", yaml_path);
        return 1;
    }

    CalibData data;
    memset(&data, 0, sizeof(data));
    data.lensType = (uint8_t)LENS_TYPE_PINHOLE; /* default to pinhole model */
    data.fx = fx;
    data.fy = fy;
    data.cx = cx;
    data.cy = cy;
    for (int i = 0; i < 5; ++i) {
        data.distCoeffs[i] = k[i];
    }

    int rc = usbcamCalibWriteFlash(video_index, SONIX_CALIB_FLASH_ADDR, data);
    if (rc != 0) {
        fprintf(stderr,
                "usbcamCalibWriteFlash failed for /dev/video%d (rc=%d, addr=0x%lX)\n",
                video_index,
                rc,
                (long)SONIX_CALIB_FLASH_ADDR);
        return 1;
    }

    printf("Calibration from '%s' written to /dev/video%d @ 0x%lX successfully.\n",
           yaml_path,
           video_index,
           (long)SONIX_CALIB_FLASH_ADDR);

    return 0;
}

static int cmd_light_set(int video_index, int value)
{
    // 查询设备支持的实际范围
    int min_val = 0, max_val = 100;
    if (v4l2_query_gain_range(video_index, &min_val, &max_val) != 0) {
        fprintf(stderr, "Failed to query gain control range for /dev/video%d\n", video_index);
        // 如果查询失败，使用默认范围0-100
    }
    
    // 验证输入值范围（0-100百分比）
    if (value < 0 || value > 100) {
        fprintf(stderr, "Error: Brightness value must be between 0 and 100 (got %d)\n", value);
        return 1;
    }
    
    // 读取设置前的值
    int before_value = 0;
    if (usbcamGetLightStatus(video_index, &before_value) == 0) {
        printf("Current light value: %d (range=[%d-%d])\n", before_value, min_val, max_val);
    }
    
    // 将百分比转换为设备实际范围
    int actual_value;
    if (max_val > min_val) {
        actual_value = min_val + (value * (max_val - min_val)) / 100;
    } else {
        actual_value = min_val;
    }
    
    // 验证转换后的值是否在有效范围内
    if (actual_value < min_val || actual_value > max_val) {
        fprintf(stderr, "Error: Calculated value %d is out of range (min=%d max=%d)\n", 
                actual_value, min_val, max_val);
        return 1;
    }
    
    printf("Setting light to %d%% (actual value: %d)...\n", value, actual_value);
    
    // 执行设置（现在会在视频流打开时设置）
    int rc = usbcamSetLightStatus(video_index, actual_value);
    if (rc != 0) {
        fprintf(stderr,
                "usbcamSetLightStatus failed for /dev/video%d (rc=%d, value=%d)\n",
                video_index,
                rc,
                actual_value);
        return 1;
    }
    
    // 等待并验证设置是否生效
    usleep(100000); // 100ms
    int after_value = 0;
    if (usbcamGetLightStatus(video_index, &after_value) == 0) {
        if (after_value != actual_value) {
            fprintf(stderr, "WARNING: Setting may not persist! Expected %d, got %d\n", 
                    actual_value, after_value);
            fprintf(stderr, "This may indicate the device requires video stream to be active.\n");
        } else {
            printf("Verification OK: value set to %d\n", after_value);
        }
    }
    
    const char* state = (actual_value == min_val) ? "OFF" : "ON";
    printf("/dev/video%d PWM light: %s (brightness=%d%%, actual=%d, range=[%d-%d])\n", 
           video_index, state, value, actual_value, min_val, max_val);
    return 0;
}

static int cmd_light_on(int video_index)
{
    return cmd_light_set(video_index, 100);
}

static int cmd_light_off(int video_index)
{
    return cmd_light_set(video_index, 0);
}

static const char* lens_type_to_string(uint8_t lens_type)
{
    switch (lens_type) {
    case LENS_TYPE_PINHOLE:
        return "pinhole";
    case LENS_TYPE_FISHEYE:
        return "fisheye";
    default:
        return "unknown";
    }
}

static int parse_lens_type(const char* s, uint8_t* out)
{
    if (!s || !out)
        return -1;

    if (strcmp(s, "pinhole") == 0 || strcmp(s, "PINHOLE") == 0) {
        *out = (uint8_t)LENS_TYPE_PINHOLE;
        return 0;
    }
    if (strcmp(s, "fisheye") == 0 || strcmp(s, "FISHEYE") == 0) {
        *out = (uint8_t)LENS_TYPE_FISHEYE;
        return 0;
    }

    return -1;
}

static int cmd_calib_read(int video_index)
{
    CalibData data;
    int rc = usbcamCalibReadFlash(video_index, SONIX_CALIB_FLASH_ADDR, &data);
    if (rc != 0) {
        fprintf(stderr,
                "usbcamCalibReadFlash failed for /dev/video%d (rc=%d, addr=0x%lX)\n",
                video_index,
                rc,
                (long)SONIX_CALIB_FLASH_ADDR);
        return 1;
    }

    printf("Calibration data from /dev/video%d @ 0x%lX:\n",
           video_index,
           (long)SONIX_CALIB_FLASH_ADDR);
    printf("  lensType  = %u (%s)\n",
           (unsigned)data.lensType,
           lens_type_to_string(data.lensType));
    printf("  fx        = %.15g\n", data.fx);
    printf("  fy        = %.15g\n", data.fy);
    printf("  cx        = %.15g\n", data.cx);
    printf("  cy        = %.15g\n", data.cy);
    printf("  distCoeffs= [%.15g, %.15g, %.15g, %.15g, %.15g]\n",
           data.distCoeffs[0],
           data.distCoeffs[1],
           data.distCoeffs[2],
           data.distCoeffs[3],
           data.distCoeffs[4]);
    printf("  checksum  = 0x%02X\n", (unsigned)data.checksum);

    return 0;
}

static int cmd_calib_write(int video_index, int argc, char** argv)
{
    if (argc < 11) {
        fprintf(stderr,
                "calib write requires: <lens_type> fx fy cx cy k0 k1 k2 k3 k4\n");
        return 1;
    }

    const char* lens_str = argv[0];
    uint8_t lens_type = 0;
    if (parse_lens_type(lens_str, &lens_type) != 0) {
        fprintf(stderr, "Unknown lens_type: %s (expected pinhole|fisheye)\n", lens_str);
        return 1;
    }

    char* end = NULL;
    double fx = strtod(argv[1], &end);
    if (!argv[1][0] || (end && *end)) {
        fprintf(stderr, "Invalid fx: %s\n", argv[1]);
        return 1;
    }
    double fy = strtod(argv[2], &end);
    if (!argv[2][0] || (end && *end)) {
        fprintf(stderr, "Invalid fy: %s\n", argv[2]);
        return 1;
    }
    double cx = strtod(argv[3], &end);
    if (!argv[3][0] || (end && *end)) {
        fprintf(stderr, "Invalid cx: %s\n", argv[3]);
        return 1;
    }
    double cy = strtod(argv[4], &end);
    if (!argv[4][0] || (end && *end)) {
        fprintf(stderr, "Invalid cy: %s\n", argv[4]);
        return 1;
    }

    double k[5];
    for (int i = 0; i < 5; ++i) {
        k[i] = strtod(argv[5 + i], &end);
        if (!argv[5 + i][0] || (end && *end)) {
            fprintf(stderr, "Invalid k%d: %s\n", i, argv[5 + i]);
            return 1;
        }
    }

    CalibData data;
    memset(&data, 0, sizeof(data));
    data.lensType = lens_type;
    data.fx = fx;
    data.fy = fy;
    data.cx = cx;
    data.cy = cy;
    for (int i = 0; i < 5; ++i) {
        data.distCoeffs[i] = k[i];
    }

    int rc = usbcamCalibWriteFlash(video_index, SONIX_CALIB_FLASH_ADDR, data);
    if (rc != 0) {
        fprintf(stderr,
                "usbcamCalibWriteFlash failed for /dev/video%d (rc=%d, addr=0x%lX)\n",
                video_index,
                rc,
                (long)SONIX_CALIB_FLASH_ADDR);
        return 1;
    }

    printf("Calibration written to /dev/video%d @ 0x%lX successfully.\n",
           video_index,
           (long)SONIX_CALIB_FLASH_ADDR);

    return 0;
}

static int cmd_fw_upgrade(int video_index, int argc, char** argv)
{
    if (argc < 1) {
        fprintf(stderr, "fw upgrade requires: <fw_path> [--preserve-calib|--partial]\n");
        return 1;
    }

    const char* fw_path = argv[0];
    const char* mode = "--preserve-calib";  // 默认模式
    if (argc >= 2) {
        mode = argv[1];
    }

    long calib_addr = SONIX_CALIB_FLASH_ADDR;
    int rc = 0;

    if (strcmp(mode, "--preserve-calib") == 0) {
        printf("Upgrading firmware with full erase (preserving calibration data)...\n");
        printf("Firmware file: %s\n", fw_path);
        printf("Calibration address: 0x%05lX\n", calib_addr);
        rc = usbcamUpgradeFwPreserveCalib(video_index, fw_path, calib_addr);
    } else if (strcmp(mode, "--partial") == 0) {
        printf("Upgrading firmware with partial erase (preserving calibration data)...\n");
        printf("Firmware file: %s\n", fw_path);
        printf("Calibration address: 0x%05lX\n", calib_addr);
        rc = usbcamUpgradeFwPartial(video_index, fw_path, calib_addr);
    } else {
        fprintf(stderr, "Unknown mode: %s (expected --preserve-calib or --partial)\n", mode);
        return 1;
    }

    if (rc == 0) {
        printf("Firmware upgrade completed successfully!\n");
        return 0;
    } else {
        fprintf(stderr, "Firmware upgrade failed with error code: %d\n", rc);
        return 1;
    }
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    int video_index = atoi(argv[1]);
    const char* category = argv[2];

    if (strcmp(category, "light") == 0) {
        if (argc < 4) {
            print_usage(argv[0]);
            return 1;
        }
        const char* sub = argv[3];
        if (strcmp(sub, "get") == 0) {
            return cmd_light_get(video_index);
        } else if (strcmp(sub, "on") == 0) {
            return cmd_light_on(video_index);
        } else if (strcmp(sub, "off") == 0) {
            return cmd_light_off(video_index);
        } else if (strcmp(sub, "set") == 0) {
            if (argc < 5) {
                print_usage(argv[0]);
                return 1;
            }
            int value = atoi(argv[4]);
            return cmd_light_set(video_index, value);
        } else {
            print_usage(argv[0]);
            return 1;
        }
    } else if (strcmp(category, "calib") == 0) {
        if (argc < 4) {
            print_usage(argv[0]);
            return 1;
        }
        const char* sub = argv[3];
        if (strcmp(sub, "read") == 0) {
            return cmd_calib_read(video_index);
        } else if (strcmp(sub, "write") == 0) {
            /* remaining args after "write" */
            return cmd_calib_write(video_index, argc - 4, &argv[4]);
        } else if (strcmp(sub, "write_yaml") == 0) {
            const char* yaml_path;
            if (argc >= 5) {
                // User provided a path
                yaml_path = argv[4];
            } else {
                // Use default path: ../calib1.yml (relative to app1 directory)
                yaml_path = "../calib1.yml";
                printf("No YAML path provided, using default: %s\n", yaml_path);
            }
            return cmd_calib_write_yaml(video_index, yaml_path);
        } else {
            print_usage(argv[0]);
            return 1;
        }
    } else if (strcmp(category, "fw") == 0) {
        if (argc < 4) {
            print_usage(argv[0]);
            return 1;
        }
        const char* sub = argv[3];
        if (strcmp(sub, "upgrade") == 0) {
            /* remaining args after "upgrade" */
            return cmd_fw_upgrade(video_index, argc - 4, &argv[4]);
        } else {
            print_usage(argv[0]);
            return 1;
        }
    } else {
        print_usage(argv[0]);
        return 1;
    }
}
