/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <M5Stack.h>

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//Are we currently connected?
boolean connected = false;
extern "C" {
    int sockfd = -1;
};

//wifi event handler
esp_err_t event_handler(void *ctx, system_event_t *event)
{
    system_event_sta_disconnected_t *disconn;
    wifi_mode_t mode;

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_DISCONNECTED:
        disconn = &event->event_info.disconnected;
        switch (disconn->reason) {
        case WIFI_REASON_AUTH_FAIL:
            Serial.println("WiFi: desconntcted after auth fail");
            break;
        default:
            // try to reconnect
            if (esp_wifi_get_mode(&mode) == ESP_OK) {
                if (mode & WIFI_MODE_STA) {
                    Serial.println("WiFi: try to reconnect...");
                    esp_wifi_connect();
                }
            }
            break;
        }
    default:
        break;
    }
    return ESP_OK;
}

// The setup routine runs once when M5Stack starts up
void setup(){

    // Initialize the M5Stack object
    M5.begin();

    // ??? analogRead makes some click noise on M5 speaker even when
    // it's muted. Looks that the speaker output pin(25) is effected by
    // electorical noise. Zero DAC output to pin 25 reduces that click
    // sound, though there is still small residual noise.
    M5.Speaker.write(0);

    // LCD display
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setCursor(20, 0);
    M5.Lcd.printf("I2S input test");

    //Connect to the WiFi network
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config;
    memset(&sta_config, 0, sizeof(sta_config));
    strcpy((char *)sta_config.sta.ssid, CONFIG_SSID);
    strcpy((char *)sta_config.sta.password, CONFIG_SSID_PASSWORD);
    sta_config.sta.bssid_set = false;
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    //ESP_ERROR_CHECK( esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );

    struct sockaddr_in saddr;
    struct sockaddr_in caddr;
    int rtn;

    Serial.println("UDP client task starting...");

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if(s < 0) {
        Serial.println("... Failed to allocate socket.");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }

    Serial.println("... allocated socket");

    memset((char *) &caddr, 0, sizeof(caddr));
    caddr.sin_family = AF_INET;
    caddr.sin_addr.s_addr = htonl(INADDR_ANY);
    caddr.sin_port = htons(CONFIG_UDP_PORT);

    memset((char *) &saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = inet_addr (CONFIG_UDP_ADDRESS);
    saddr.sin_port = htons(CONFIG_UDP_PORT);

    rtn = bind (s, (struct sockaddr *)&caddr, sizeof(caddr));
    if(rtn < 0) {
        Serial.println("... Failed to bind socket.");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }
    rtn = connect(s, (struct sockaddr *) &saddr, sizeof(saddr));
    if (rtn < 0) {
        Serial.println("... Failed to connect socket.");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }

    sockfd = s;
    connected = true;
}

extern "C" {
    void i2s_task(void *arg);
#if CONFIG_I2S_DISPLAY_FFT
    extern float spectrum[512];
#endif
}

int count = 0;

// The loop routine runs over and over again forever
void loop() {
    M5.Lcd.setCursor(160, 0);
    M5.Lcd.printf("%7d ms", 100*count);
    count++;

#if CONFIG_I2S_DISPLAY_FFT
    M5.Lcd.fillRect(20, 40, 280, 190, TFT_NAVY);

    for (int i = 0; i < 256; i++) {
        int16_t ix = 30 + i;
        int16_t iy = 150 - (int16_t) (8*spectrum[i]);
        if (iy < 40)
            iy = 40;
        if (iy > 220)
            iy = 220;

        M5.Lcd.drawPixel(ix, iy, GREEN);
    }
#endif

    M5.update();
    delay(100);
}

// The arduino task
void loopTask(void *pvParameters)
{
    setup();

    for(;;) {
        //micros(); //update overflow
        loop();
    }
}

extern "C" void app_main()
{
    initArduino();

    xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreate(i2s_task, "i2s_task", 8192, NULL, 1, NULL);
}
