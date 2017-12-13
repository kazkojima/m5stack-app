/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#include "MadgwickAHRS.h"
#include <math.h>

#define  ROTATION_YAW	0

#define I2C_MASTER_SCL_IO               22
#define I2C_MASTER_SDA_IO               21
#define I2C_MASTER_NUM                  I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE       0
#define I2C_MASTER_RX_BUF_DISABLE       0
#define I2C_MASTER_FREQ_HZ              400000

#define I2C_PORT                        I2C_MASTER_NUM
#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    0x1
#define ACK_CHECK_DIS                   0x0
#define ACK_VAL                         0x0
#define NACK_VAL                        0x1

void i2c_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE,
                       0);
}

// MPU9250
#define MPU9250_I2C_ADDR 0x68
#define MPU9250_ID      0x71

// MPU9250 registers
#define XG_OFFSET_H     0x13
#define XG_OFFSET_L     0x14
#define YG_OFFSET_H     0x15
#define YG_OFFSET_L     0x16
#define ZG_OFFSET_H     0x17
#define ZG_OFFSET_L     0x18

#define SMPLRT_DIV      0x19
#define MPU_CONFIG      0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_CONFIG2   0x1D

#define FIFO_EN         0x23

#define I2C_MST_CTRL    0x24
#define I2C_SLV0_ADDR   0x25
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_CTRL   0x27

#define I2C_SLV4_CTRL   0x34

#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A

#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48

#define EXT_SENS_DATA_00 0x49
#define I2C_SLV0_DO     0x63
#define I2C_MST_DELAY_CTRL 0x67

#define USER_CTRL       0x6A
#define PWR_MGMT_1      0x6B
#define PWR_MGMT_2      0x6C

#define FIFO_COUNTH     0x72
#define FIFO_COUNTL     0x73
#define FIFO_R_W        0x74

#define WHO_IM_I        0x75

#define XA_OFFSET_H     0x77
#define XA_OFFSET_L     0x78
#define YA_OFFSET_H     0x7A
#define YA_OFFSET_L     0x7B
#define ZA_OFFSET_H     0x7D
#define ZA_OFFSET_L     0x7E

// AK8963
#define AK8963_I2C_ADDR 0x0c
#define AK8963_ID       0x48

/* AK8963 registers */
#define AK8963_WIA      0x00
#define AK8963_ST1      0x02
#define AK8963_HXL      0x03
#define AK8963_CNTL1    0x0A
#define AK8963_CNTL2    0x0B
#define AK8963_ASAX     0x10

#define GRAVITY_MSS     9.80665f

#define FILTER_CONVERGE_COUNT 8000

// accelerometer scaling for 16g range
#define MPU9250_ACCEL_SCALE_1G    (GRAVITY_MSS / 2048.0f)

/*
 *  PS-MPU-9250A-00.pdf, page 8, lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
static const float GYRO_SCALE = 0.0174532f / 16.4f;

static const float TEMP_SCALE = 1.0f / 333.87f;
#define TEMP_OFFSET 21.0f

#define AK8963_MILLIGAUSS_SCALE 10.0f
static const float ADC_16BIT_RESOLUTION = 0.15f;

// MPU9250 IMU data are big endian
#define be16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
// AK8963 data are little endian
#define le16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx+1] << 8) | v[2*idx]))

extern spi_device_handle_t spi_a;

static uint8_t mpu9250_read(uint8_t reg)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_I2C_ADDR << 1 )|WRITE_BIT,
			  ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return 0;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_I2C_ADDR << 1 )|READ_BIT,
			  ACK_CHECK_EN);
    uint8_t rv = 0;
    i2c_master_read_byte(cmd, &rv, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        rv = 0;
    }
    return rv;
}

static esp_err_t mpu9250_write(uint8_t reg, uint8_t val)
{
    uint8_t d = val;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_I2C_ADDR << 1)|WRITE_BIT,
			  ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, &d, 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu9250_readn(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_I2C_ADDR << 1)|WRITE_BIT,
			  ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_I2C_ADDR << 1)|READ_BIT,
			  ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static uint8_t ak8963_read(uint8_t reg)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK8963_I2C_ADDR << 1 )|WRITE_BIT,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return 0;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK8963_I2C_ADDR << 1 )|READ_BIT,
                          ACK_CHECK_EN);
    uint8_t rv = 0;
    i2c_master_read_byte(cmd, &rv, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        rv = 0;
    }
    return rv;
}

static esp_err_t ak8963_write(uint8_t reg, uint8_t val)
{
    uint8_t d = val;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK8963_I2C_ADDR << 1)|WRITE_BIT,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, &d, 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t ak8963_readn(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK8963_I2C_ADDR << 1)|WRITE_BIT,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AK8963_I2C_ADDR << 1)|READ_BIT,
                          ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static bool mpu9250_ready(void)
{
    uint8_t val = mpu9250_read(INT_STATUS);
    return (val & 1);
}

struct sample {
    uint8_t d[14];
};

#if 0
static bool mpu9250_read_sample(struct sample *rx)
{
    esp_err_t rv;
    rv = mpu9250_readn(ACCEL_XOUT_H, (uint8_t *)rx, sizeof(struct sample));
    return (rv == ESP_OK);
}
#endif

static int mpu9250_fifo_count(void)
{
    uint8_t c[2];
    esp_err_t rv;
    rv = mpu9250_readn(FIFO_COUNTH, c, 2);
    return (rv == ESP_OK) ? be16_val(c, 0) : 0;
}

static bool mpu9250_read_fifo(struct sample *rx)
{
    esp_err_t rv;
    rv = mpu9250_readn(FIFO_R_W, (uint8_t *)rx, sizeof(struct sample));
    return (rv == ESP_OK);
}

// Check fifo entry with temperature
static bool check_fifo(int t)
{
    static int raw;
    static bool cached = false;
    if (cached && t - raw > -340 && t - raw < 340) {
        return true;
    }
    uint8_t temp[2];
    if (ESP_OK == mpu9250_readn(TEMP_OUT_H, temp, 2)) {
        raw = be16_val(temp, 0);
        cached = true;
    }
    return (t - raw > -340 && t - raw < 340);
}

static void mpu9250_fifo_reset(void)
{
    uint8_t val = mpu9250_read(USER_CTRL);
    val &= ~0x44;
    mpu9250_write(FIFO_EN, 0);
    mpu9250_write(USER_CTRL, val);
    mpu9250_write(USER_CTRL, val|0x04);
    mpu9250_write(USER_CTRL, val|0x40);
    // All except external sensors
    mpu9250_write(FIFO_EN, 0xf8);
    vTaskDelay(1/portTICK_PERIOD_MS);
}

static void mpu9250_start(void)
{
    mpu9250_write(PWR_MGMT_2, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // bit0 1: Set LPF to 184Hz 0: No LPF, bit6 Stop if fifo full
    mpu9250_write(MPU_CONFIG, (1<<6) | 1);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Sample rate 1000Hz
    mpu9250_write(SMPLRT_DIV, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Gyro 2000dps
    mpu9250_write(GYRO_CONFIG, 3<<3);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Accel full scale 16g
    mpu9250_write(ACCEL_CONFIG, 3<<3);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Set LPF to 218Hz BW
    mpu9250_write(ACCEL_CONFIG2, 1);
    vTaskDelay(1/portTICK_PERIOD_MS);

    uint8_t val;
    // INT enable on RDY
    mpu9250_write(INT_ENABLE, 1);
    vTaskDelay(1/portTICK_PERIOD_MS);

    val = mpu9250_read(INT_PIN_CFG);
    val |= 0x30;
    mpu9250_write(INT_PIN_CFG, val);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Looks that enabling DMP without blob causes continuous fifo sync errors
#if 0
    val = mpu9250_read(USER_CTRL);
    mpu9250_write(USER_CTRL, val | (1<<7));
    vTaskDelay(1/portTICK_PERIOD_MS);
#endif
}

struct ak_sample {
    uint8_t d[6];
    uint8_t st2;
};

struct ak_asa {
    uint8_t a[3];
};

static struct ak_asa ak8963_asa;
static float ak8963_calib[3];

static void ak8963_start(void)
{
    // Reset
    // ak8963_write(AK8963_CNTL2, 0x01);

    // Calibrate - fuse, 16-bit adc
    ak8963_write(AK8963_CNTL1, 0x1f);
    vTaskDelay(10/portTICK_PERIOD_MS);

    ak8963_readn(AK8963_ASAX, (uint8_t *)&ak8963_asa, sizeof(ak8963_asa));
    vTaskDelay(10/portTICK_PERIOD_MS);

    for (int i = 0; i < 3; i++) {
        float data = ak8963_asa.a[i];
        // factory sensitivity
        ak8963_calib[i] = ((data - 128) / 256 + 1);
        // adjust by ADC sensitivity and convert to milligauss
        ak8963_calib[i] *= ADC_16BIT_RESOLUTION * AK8963_MILLIGAUSS_SCALE;
    }

    // Setup mode - continuous mode 2, 16-bit adc
    ak8963_write(AK8963_CNTL1, 0x16);
    vTaskDelay(10/portTICK_PERIOD_MS);
    // Start measurement
}

// For compass
#if CONFIG_ZHINANCHE
#define COMPASS_MODE_ZHINANCHE 1
#else
#define COMPASS_MODE_ZHINANCHE 0
#endif
static float m5_mag_offset[3] = { -5780.0, 4020.0, -3155.0 };
extern xQueueHandle att_queue;
  
void imu_task(void *arg)
{
    vTaskDelay(100/portTICK_PERIOD_MS);
    i2c_init();
    vTaskDelay(100/portTICK_PERIOD_MS);

    uint8_t rv;
    rv = mpu9250_read(WHO_IM_I);
    if (rv != MPU9250_ID) {
        printf("MPU9250: Wrong id: %02x\n", rv);
        vTaskDelete(NULL);
    }

    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        // Disable master I2C here
        if ((rv = mpu9250_read(USER_CTRL)) & (1<<5)) {
            mpu9250_write(USER_CTRL, rv &~ (1<<5));
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        // Reset
        mpu9250_write(PWR_MGMT_1, 0x80);
        vTaskDelay(100 / portTICK_PERIOD_MS);

#if 0
        // Disable I2C interface
        mpu9250_write(USER_CTRL, 0x10);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Enable master I2C access to AK8963
        mpu9250_write(INT_PIN_CFG, 0x02);
#else
        // Enable bypassing I2C access to AK8963
        mpu9250_write(INT_PIN_CFG, 0x22);
        mpu9250_write(INT_ENABLE, 0x01);
        vTaskDelay(100 / portTICK_PERIOD_MS);
#endif

        // Wake up with appropriate clock
        mpu9250_write(PWR_MGMT_1, 0x03);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        if (mpu9250_read(PWR_MGMT_1) == 0x03)
            break;

        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (mpu9250_ready())
            break;
    }

    if (tries == 5) {
        printf("Failed to boot MPU9250 5 times");
        vTaskDelete(NULL);
    }

    mpu9250_start();
    mpu9250_fifo_reset();
    
    rv = ak8963_read(AK8963_WIA);
    if (rv != AK8963_ID) {
        printf("AK8963: Wrong id: %02x\n", rv);
        vTaskDelete(NULL);
    }
    vTaskDelay(10/portTICK_PERIOD_MS);

    ak8963_start();

    struct sample rx;
    struct ak_sample akrx;
    int count = 0;
    float gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0, mx, my, mz;
    float temp;

    while (1) {
        int fifo_count = mpu9250_fifo_count();
        if (fifo_count == 0) {
            vTaskDelay(1 / portTICK_PERIOD_MS);
            continue;
        }
	
        int n_sample = fifo_count / sizeof(rx);
        bool try_compass = false;

        while(n_sample--) {
            mpu9250_read_fifo(&rx);

            int16_t t = be16_val(rx.d, 3);
            //printf("temp %f\n", t/340.0+21);
            if (!check_fifo(t)) {
                mpu9250_fifo_reset();
                printf("temp reset fifo %04x %d\n", t, fifo_count);
                break;
            }

            // adjust and serialize floats into packet bytes
            // skew accel/gyro frames so to match AK8963 NED frame
            union { float f; uint8_t bytes[sizeof(float)];} ux, uy, uz;
#if (ROTATION_YAW == 0)
            ux.f = ((float)be16_val(rx.d, 1)) * MPU9250_ACCEL_SCALE_1G;
            uy.f = ((float)be16_val(rx.d, 0)) * MPU9250_ACCEL_SCALE_1G;
            uz.f = -((float)be16_val(rx.d, 2)) * MPU9250_ACCEL_SCALE_1G;
#elif (ROTATION_YAW == 90)
            ux.f = -((float)be16_val(rx.d, 0)) * MPU9250_ACCEL_SCALE_1G;
            uy.f = ((float)be16_val(rx.d, 1)) * MPU9250_ACCEL_SCALE_1G;
            uz.f = -((float)be16_val(rx.d, 2)) * MPU9250_ACCEL_SCALE_1G;
#elif (ROTATION_YAW == 180)
            ux.f = -((float)be16_val(rx.d, 1)) * MPU9250_ACCEL_SCALE_1G;
            uy.f = -((float)be16_val(rx.d, 0)) * MPU9250_ACCEL_SCALE_1G;
            uz.f = -((float)be16_val(rx.d, 2)) * MPU9250_ACCEL_SCALE_1G;
#elif (ROTATION_YAW == 270)
            ux.f = ((float)be16_val(rx.d, 0)) * MPU9250_ACCEL_SCALE_1G;
            uy.f = -((float)be16_val(rx.d, 1)) * MPU9250_ACCEL_SCALE_1G;
            uz.f = -((float)be16_val(rx.d, 2)) * MPU9250_ACCEL_SCALE_1G;
#else
#error "bad ROTATION_YAW value"
#endif
            ax = ux.f; ay = uy.f; az = uz.f;

            union { float f; uint8_t bytes[sizeof(float)];} ut;
            ut.f = ((float)be16_val(rx.d, 3)) * TEMP_SCALE + TEMP_OFFSET;
            temp = ut.f;

#if (ROTATION_YAW == 0)
            ux.f = ((float)be16_val(rx.d, 5)) * GYRO_SCALE;
            uy.f = ((float)be16_val(rx.d, 4)) * GYRO_SCALE;
            uz.f = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#elif (ROTATION_YAW == 90)
            ux.f = -((float)be16_val(rx.d, 4)) * GYRO_SCALE;
            uy.f = ((float)be16_val(rx.d, 5)) * GYRO_SCALE;
            uz.f = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#elif (ROTATION_YAW == 180)
            ux.f = -((float)be16_val(rx.d, 5)) * GYRO_SCALE;
            uy.f = -((float)be16_val(rx.d, 4)) * GYRO_SCALE;
            uz.f = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#elif (ROTATION_YAW == 270)
            ux.f = ((float)be16_val(rx.d, 4)) * GYRO_SCALE;
            uy.f = -((float)be16_val(rx.d, 5)) * GYRO_SCALE;
            uz.f = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#else
#error "bad ROTATION_YAW value"
#endif
            gx = ux.f; gy = uy.f; gz = uz.f;

            if ((count++ % 10) == 0) {
                try_compass = true;
            }
            MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
            //printf("ax: %f ay: %f az: %f temp: %f\n", ax, ay, az, temp);
            //printf("gx: %f gy: %f gz: %f\n", gx, gy, gz);
        }

        if ((count++ % 10) == 0 || try_compass) {
            if (!(ak8963_read(AK8963_ST1) & 0x1)) {
                continue;
            }
            ak8963_readn(AK8963_HXL, (uint8_t *)&akrx, sizeof(akrx));
            // skip if overflow
            if (akrx.st2 & 0x08) {
                continue;
            }
            // adjust and serialize floats into packet bytes
            union { float f; uint8_t bytes[sizeof(float)];} ux, uy, uz;
#if (ROTATION_YAW == 0)
            ux.f = ((float)le16_val(akrx.d, 0)) * ak8963_calib[0];
            uy.f = ((float)le16_val(akrx.d, 1)) * ak8963_calib[1];
            uz.f = ((float)le16_val(akrx.d, 2)) * ak8963_calib[2];
#elif (ROTATION_YAW == 90)
            ux.f = -((float)le16_val(akrx.d, 1)) * ak8963_calib[0];
            uy.f = ((float)le16_val(akrx.d, 0)) * ak8963_calib[1];
            uz.f = ((float)le16_val(akrx.d, 2)) * ak8963_calib[2];
#elif (ROTATION_YAW == 180)
            ux.f = -((float)le16_val(akrx.d, 0)) * ak8963_calib[0];
            uy.f = -((float)le16_val(akrx.d, 1)) * ak8963_calib[1];
            uz.f = ((float)le16_val(akrx.d, 2)) * ak8963_calib[2];
#elif (ROTATION_YAW == 270)
            ux.f = ((float)le16_val(akrx.d, 1)) * ak8963_calib[0];
            uy.f = -((float)le16_val(akrx.d, 0)) * ak8963_calib[1];
            uz.f = ((float)le16_val(akrx.d, 2)) * ak8963_calib[2];
#else
#error "bad ROTATION_YAW value"
#endif
            mx = ux.f; my = uy.f; mz = uz.f;
            mx -= m5_mag_offset[0];
            my -= m5_mag_offset[1];
            mz -= m5_mag_offset[2];
            if (count++ < FILTER_CONVERGE_COUNT) {
                beta = 32.0f;
                MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
            } else {
                beta = 1.0f;
                if (!COMPASS_MODE_ZHINANCHE)
                    MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
            }
            //printf ("q0 %7.3f q1 %7.3f q2 %7.3f q3 %7.3f\n", q0, q1, q2, q3);
            //printf("mx: %f my: %f mz: %f\n", mx, my, mz);

            // These are linear approximations which would be enough for
            // our purpose.
            float att[4];
            att[0] = -(q0*q1+q3*q2);
            att[1] = -(q0*q2-q3*q1);
            att[2] = (0.5-(q2*q2+q3*q3));
            att[3] = -(q0*q3+q1*q2);
            //printf ("roll %7.5f pitch %7.5f\n", att[0], att[1]);
            if (xQueueSend(att_queue, &att[0], 0) != pdTRUE) {
                printf("fail to queue attitude\n");
            }
        }
    }
}
