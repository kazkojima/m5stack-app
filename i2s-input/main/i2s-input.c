/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#define I2SNDMABUF		4
#define I2SBUFLEN		(32*2)

#define SAMPLE_RATE     (CONFIG_I2S_SAMPLE_RATE)
#define I2S_NUM         (0)

static void i2s_init(void)
{
    esp_err_t ret;
    i2s_config_t i2s_config =
        {
         .mode = I2S_MODE_MASTER | I2S_MODE_RX,
         .sample_rate = SAMPLE_RATE,
         .bits_per_sample = I2S_BITS_PER_SAMPLE_24BIT,
#if CONFIG_I2S_STEREO
         .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
#else
         .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
#endif
         .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
         .dma_buf_count = I2SNDMABUF,
         .dma_buf_len = I2SBUFLEN,
         .use_apll = false,
         .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        };
    i2s_pin_config_t pin_config =
        {
         .bck_io_num = CONFIG_I2S_BCK_GPIO,
         .ws_io_num = CONFIG_I2S_WS_GPIO,
         .data_out_num = -1,
         .data_in_num = CONFIG_I2S_DATAIN_GPIO,
    };

    ret = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    assert(ret==ESP_OK);
    ret = i2s_set_pin(I2S_NUM, &pin_config);
    assert(ret==ESP_OK);
}

extern int sockfd;

#if CONFIG_I2S_DISPLAY_FFT
// Definition for FFT. Fixed numbers used. See fftsg.c for details.
#define I2S_BUFSIZE	4096
#define I2S_BUFSIZEAQRT	64
static uint32_t fbuf[I2S_BUFSIZE/2+1];

#define NMAX I2S_BUFSIZE/2
#define NMAXSQRT I2S_BUFSIZEAQRT
static int cfdt_ip[NMAXSQRT + 2];
static float cfdt_w[NMAX * 5 / 4];
float spectrum[NMAX/4];

extern void makewt(int nw, int *ip, float *w);
extern void cdft(int n, int isgn, float *a, int *ip, float *w);
extern float log10f (float x);
#endif

void i2s_task(void *arg)
{
    esp_err_t ret;

    // Prepare cfdt_w table.
    makewt (NMAX >> 2, cfdt_ip, cfdt_w);

    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    i2s_init();
    printf("I2S initialized\n");

#if CONFIG_I2S_MCLK_GPIO1
    REG_WRITE(PIN_CTRL, CLK_OUT3_M);
    gpio_iomux_out(1, FUNC_U0TXD_CLK_OUT3, false);
#endif

    float *ap = (float *)(void *) fbuf;
    int i = 0;
 
    uint32_t buf[I2SBUFLEN*sizeof(uint32_t)];
    while (1) {
        size_t size = I2SBUFLEN*sizeof(uint32_t);
        size_t bytes_read;
        ret = i2s_read(I2S_NUM, &buf[0], size, &bytes_read, portMAX_DELAY);
        if (ret == ESP_OK && sockfd >= 0)
            send(sockfd, &buf[0], size, 0);

#if CONFIG_I2S_DISPLAY_FFT
        float *tp;
        if (i == 0)
            tp = ap;
        for (int j = 0; j < I2SBUFLEN; j++, i++) {
            *tp++ =  ((float) (buf[j] >> 8))/(1<<23);
            *tp++ = 0;
        }
        if (i == NMAX/2) {
            cdft (NMAX, 1, ap, cfdt_ip, cfdt_w);
            // Gen power spectrum
            for (tp = ap, i = 0; i < NMAX/4; i++) {
                float re, im;
                re = *tp++;
                im = *tp++;
                spectrum[i] = log10f (re * re + im * im);
            }
            i = 0;
        }
#endif
    }
}
