/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <M5Stack.h>
#include <WiFi.h>

#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

const char *networkName = CONFIG_SSID;
const char *networkPswd = CONFIG_SSID_PASSWORD;

//APM server IP address
const char *tcpAddress = CONFIG_SERVER_ADDRESS;
const int tcpPort = CONFIG_TCP_PORT;

//Are we currently connected?
boolean connected = false;

//TCP client
WiFiClient client;

//wifi event handler
static void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());

          // Start device display with ID of sensor
          M5.Lcd.fillRect(20, 0, 300, 16, BLACK);
          M5.Lcd.setTextColor(GREEN, BLACK);
          M5.Lcd.setCursor(20, 0);
          M5.Lcd.print("WiFi connected! IP address: ");
          M5.Lcd.print(WiFi.localIP());

          if(client.connect(tcpAddress, tcpPort))
              connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");

          // Start device display with ID of sensor
          M5.Lcd.fillRect(20, 0, 300, 16, BLACK);
          M5.Lcd.setTextColor(RED, BLACK);
          M5.Lcd.setCursor(20, 0);
          M5.Lcd.print("WiFi lost connection");

         connected = false;
          break;
      default:
          break;
    }
}

static void connectToWiFi(const char * ssid, const char * pwd)
{
    Serial.println("Connecting to WiFi network: " + String(ssid));

    // delete old config
    WiFi.disconnect(true);
    //register event handler
    WiFi.onEvent(WiFiEvent);
  
    //Initiate connection
    WiFi.begin(ssid, pwd);

    Serial.println("Waiting for WIFI connection...");
}

// The setup routine runs once when M5Stack starts up
void setup(){

    // Initialize the M5Stack object
    M5.begin();

    //M5.Speaker.mute();
    // ??? analogRead makes some click noise on M5 speaker even when
    // it's muted. Looks that the speaker output pin(25) is effected by
    // electorical noise. Zero DAC output to pin 25 reduces that click
    // sound, though there is still small residual noise.
    M5.Speaker.write(0);

    // LCD display
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setCursor(20, 0);
    M5.Lcd.printf("M5 Attitude Identifier by 3DGA");

    //Connect to the WiFi network
    connectToWiFi(networkName, networkPswd);
}

// Draw HID

void applyRotor (float v0, float vi, float vj, float vk,
		 float x0, float x1, float x2,
		 float& xp0, float& xp1, float& xp2)
{
  xp0 = -vj*(vj*x0 - vi*x1 + v0*x2) + vk*(-vk*x0 + v0*x1 + vi*x2)
    + v0*(v0*x0 + vk*x1 - vj*x2) + vi*(vi*x0 + vj*x1 + vk*x2);
  xp1 = vi*(vj*x0 - vi*x1 + v0*x2) + v0*(-vk*x0 + v0*x1 + vi*x2)
    - vk*(v0*x0 + vk*x1 - vj*x2) + vj*(vi*x0 + vj*x1 + vk*x2);
  xp2 = v0*(vj*x0 - vi*x1 + v0*x2) - vi*(-vk*x0 + v0*x1 + vi*x2)
    + vj*(v0*x0 + vk*x1 - vj*x2) + vk*(vi*x0 + vj*x1 + vk*x2);
}

static int16_t last[2][2][2];

void
drawHID (float v0, float vi, float vj, float vk)
{
    float ex, ey, ez, l0x, l0y, l0z, l1x, l1y, l1z;
    applyRotor (v0, -vi, -vj, -vk, 0, 1, 0, ex, ey, ez);
    //printf("E %2.5f %2.5f %2.5f\n", ex, ey, ez);
    applyRotor (v0, -vi, -vj, -vk, 1, 0, 0, l0x, l0y, l0z);
    //printf("0 %2.5f %2.5f %2.5f\n", l0x, l0y, l0z);
    applyRotor (v0, -vi, -vj, -vk, 0, 0, 1, l1x, l1y, l1z);
    //printf("1 %2.5f %2.5f %2.5f\n", l1x, l1y, l1z);
    float pt[2][2];
    int idx;
    //M5.Lcd.fillRect(60, 40, 204, 200, BLACK);
    if (ey > 0.01 || ey < -0.01) {
        float x = ex/ey, y = ez/ey;
        idx = 0;
        if (1||l0y > 0.01 || l0y < -0.01) {
            float ux = l0x, uy = l0z;
            if (ux > 0.01 || ux < -0.01) {
                float t, c;
                t = (1 - x)/ux;
                c = y + t*uy;
                if (c >= -1 && c <= 1) {
                    pt[idx][0] = 1, pt[idx][1] = c;
                    idx++;
                }
                t = (-1 - x)/ux;
                c = y + t*uy;
                if (c >= -1 && c <= 1) {
                    pt[idx][0] = -1, pt[idx][1] = c;
                    idx++;
                }
            }
            if (uy > 0.01 || uy < -0.01) {
                float t, c;
                t = (1 - y)/uy;
                c = x + t*ux;
                if (c > -1 && c < 1) {
                    pt[idx][0] = c, pt[idx][1] = 1;
                    idx++;
                }
                t = (-1 - y)/uy;
                c = x + t*ux;
                if (c > -1 && c < 1) {
                    pt[idx][0] = c, pt[idx][1] = -1;
                    idx++;
                }
            }
            if (idx == 2) {
                // convert
                int16_t x0 = pt[0][0]*100 + 160;
                int16_t y0 = -pt[0][1]*100 + 140;
                int16_t x1 = pt[1][0]*100 + 160;
                int16_t y1 = -pt[1][1]*100 + 140;
                if (last[0][0][0] != x0 || last[0][0][1] != y0
                    || last[0][1][0] != x1 || last[0][1][1] != y1) {
                    M5.Lcd.drawLine(last[0][0][0], last[0][0][1],
                                    last[0][1][0], last[0][1][1], BLACK);
                    M5.Lcd.drawLine(x0, y0, x1, y1, GREEN);
                    last[0][0][0] = x0;
                    last[0][0][1] = y0;
                    last[0][1][0] = x1;
                    last[0][1][1] = y1;
                }
            }
        }
        l0x = l1x;
        l0y = l1y;
        l0z = l1z;
        idx = 0;
        if (1||l0x > 0.01 || l0x < -0.01) {
            float ux = l0x, uy = l0z;
            if (ux > 0.01 || ux < -0.01) {
                float t, c;
                t = (1 - x)/ux;
                c = y + t*uy;
                if (c >= -1 && c <= 1) {
                    pt[idx][0] = 1, pt[idx][1] = c;
                    idx++;
                }
                t = (-1 - x)/ux;
                c = y + t*uy;
                if (c >= -1 && c <= 1) {
                    pt[idx][0] = -1, pt[idx][1] = c;
                    idx++;
                }
            }
            if (uy > 0.01 || uy < -0.01) {
                float t, c;
                t = (1 - y)/uy;
                c = x + t*ux;
                if (c > -1 && c < 1) {
                    pt[idx][0] = c, pt[idx][1] = 1;
                    idx++;
                }
                t = (-1 - y)/uy;
                c = x + t*ux;
                if (c > -1 && c < 1) {
                    pt[idx][0] = c, pt[idx][1] = -1;
                    idx++;
                }
            }
            if (idx == 2) {
                // convert
                int16_t x0 = pt[0][0]*100 + 160;
                int16_t y0 = -pt[0][1]*100 + 140;
                int16_t x1 = pt[1][0]*100 + 160;
                int16_t y1 = -pt[1][1]*100 + 140;
                if (last[1][0][0] != x0 || last[1][0][1] != y0
                    || last[1][1][0] != x1 || last[1][1][1] != y1) {
                    M5.Lcd.drawLine(last[1][0][0], last[1][0][1],
                                    last[1][1][0], last[1][1][1], BLACK);
                    M5.Lcd.drawLine(x0, y0, x1, y1, WHITE);
                    last[1][0][0] = x0;
                    last[1][0][1] = y0;
                    last[1][1][0] = x1;
                    last[1][1][1] = y1;
                }
            }
        }
    }
}

// C objects

extern "C" {
    bool trim_mode = false;
    extern float m5_mag_offset[3];
    extern float mx_raw_min, mx_raw_max;
    extern float my_raw_min, my_raw_max;
    extern float mz_raw_min, mz_raw_max;
    extern float mx_adj, my_adj, mz_adj;

    extern float filter_gain;

    xQueueHandle att_queue = NULL;
}

char buf[256];
int loop_count = 0;
// The loop routine runs over and over again forever
void loop() {
    float att[4];
    if (xQueueReceive(att_queue, &att[0], 0) == pdTRUE) {
        if (client.connected()) {
            int len = sprintf (buf,
                               "S=%2.3f+(%2.3f)*e2^e3+(%2.3f)*e3^e1+(%2.3f)*e1^e2;$\n",
                               att[0], att[1], att[2], att[3]);
            if (len > 0) {
                client.write (buf, len);
            }
        }

        // Draw HID
        drawHID (att[0], att[1], att[2], att[3]);
    }
    if (!client.connected()) {
        client.stop();
    }

    if ((++loop_count % 500) == 0) {
        M5.Lcd.fillRect(40, 20, 280, 16, BLACK);
        M5.Lcd.setTextColor(CYAN, BLACK);
        M5.Lcd.setCursor(40, 20);
        M5.Lcd.printf("Compass: %3.1f %3.1f %3.1f (NED mgauss)",
                      mx_adj, my_adj, mz_adj);
    }

    M5.update();
    delay(1);
}

// Guess compass offset values

int mag_calib_state = 0;
#define MAG_HOR CONFIG_MAG_H_INTENSITY
#define MAG_VER CONFIG_MAG_V_INTENSITY
// AK8963 range +- 4900uT
#define MAG_LIM 49000

bool guess_mag_offset(void)
{
    bool changed = false;
    if (mag_calib_state == 0
        && mx_raw_max - mx_raw_min > 2 * MAG_HOR * 0.8
        && my_raw_max - my_raw_min > 2 * MAG_HOR * 0.8) {
        //Serial.printf("mx_off %d\n\r", (int)(mx_raw_min+mx_raw_max)/2);
        //Serial.printf("my_off %d\n\r", (int)(my_raw_min+my_raw_max)/2);
        m5_mag_offset[0] = (mx_raw_min + mx_raw_max)/2;
        m5_mag_offset[1] = (my_raw_min + my_raw_max)/2;
        mag_calib_state = 1;
        changed = true;
    }
    if (mag_calib_state == 1
        && mz_raw_max - mz_raw_min > 2 * MAG_VER * 0.8) {
        //Serial.printf("mz_off\n\r", (int)(mz_raw_min+mz_raw_max)/2);
        m5_mag_offset[2] = (mz_raw_min + mz_raw_max)/2;
        mag_calib_state = 2;
        changed = true;
    }
    return changed;
}

enum adjust {
    ADJUST_TCP = 0,
    ADJUST_ZHINANCHE,
    ADJUST_GAIN,
    ADJUST_MAG_VAR,
    ADJUST_MAG_CALIB,
    ADJUST_NVS
};
#define N_ADJUST 6
int16_t adjust[N_ADJUST];
const struct { int16_t min; int16_t max; int16_t f;} adjust_range[N_ADJUST] = {
    { 0, 1, 0 },
    { 0, 1, 0 },
    { 0, 40, 10 },
    { -90, 90, 0 },
    { 0, 0, 0 },
    { 0, 0, 0 },
};
const char *adjust_string[N_ADJUST] = {
    "Enable TCP",
    "Zhinanche mode",
    "Fuse gain",
    "Mag variation",
    "Mag calibration",
    "Load/Save",
};
const char *var[] = {
    "adj_0", "adj_1", "adj_2", "adj_3", "adj_4", "adj_5", "adj_6", };

// Alternative loop for configuration mode
void loop_conf() {
    static bool update = true;
    static int adjust_index = 0;
    if (M5.BtnB.wasPressed()) {
        adjust_index = (adjust_index + 1) % N_ADJUST;
        update = true;
    }

    if (adjust_index == ADJUST_MAG_CALIB) {
        if (M5.BtnA.wasPressed()) {
            // restert calib
            mx_raw_min = my_raw_min = mz_raw_min = MAG_LIM;
            mx_raw_max = my_raw_max = mz_raw_max = -MAG_LIM;
            mag_calib_state = 0;
            update = true;
        }
        update = (update || guess_mag_offset());
    } else if (adjust_index == ADJUST_NVS) {
        nvs_handle storage_handle;
        esp_err_t err;
        if (M5.BtnA.wasPressed()) {
            // Load
            err = nvs_open("storage", NVS_READWRITE, &storage_handle);
            if (err != ESP_OK) {
                Serial.println("NVS can't be opened");
            } else {
                // none ATM
            }
            nvs_close(storage_handle);
            update = true;
        }
        if (M5.BtnC.wasPressed()) {
            // Save
            err = nvs_open("storage", NVS_READWRITE, &storage_handle);
            if (err != ESP_OK) {
                Serial.println("NVS can't be opened");
            } else {
                int32_t v;
                for (int i = 0; i < N_ADJUST - 2; i++) {
                    v = adjust[i];
                    err = nvs_set_i32(storage_handle, var[i], v);
                    if (err != ESP_OK) {
                        nvs_commit(storage_handle);
                    }
                }
                if (mag_calib_state == 2) {
                    v = (int32_t)m5_mag_offset[0];
                    err = nvs_set_i32(storage_handle, "mx_off", v);
                    if (err != ESP_OK) {
                        nvs_commit(storage_handle);
                    }
                    v = (int32_t)m5_mag_offset[1];
                    err = nvs_set_i32(storage_handle, "my_off", v);
                    if (err != ESP_OK) {
                        nvs_commit(storage_handle);
                    }
                    v = (int32_t)m5_mag_offset[2];
                    err = nvs_set_i32(storage_handle, "mz_off", v);
                    if (err != ESP_OK) {
                        nvs_commit(storage_handle);
                    }
                }
            }
            nvs_close(storage_handle);
            update = true;
        }
    } else {
        if (M5.BtnC.wasPressed()) {
            if (adjust[adjust_index] < adjust_range[adjust_index].max) {
                adjust[adjust_index]++;
                update = true;
            }
        }
        if (M5.BtnA.wasPressed()) {
            if (adjust[adjust_index] > adjust_range[adjust_index].min) {
                adjust[adjust_index]--;
                update = true;
            }
        }
     }

    if (update) {
        for (int i = 0; i < N_ADJUST; i++) {
            if (adjust_index == i) {
                M5.Lcd.setTextColor(WHITE, BLACK);
            } else {
                M5.Lcd.setTextColor(TFT_DARKGREY, BLACK);
            }
            M5.Lcd.setCursor(30, 28+i*24);
            M5.Lcd.printf(adjust_string[i]);
            M5.Lcd.fillRect(140, 28+i*24, 200, 16, BLACK);
            if (adjust_index == i) {
                M5.Lcd.setTextColor(GREEN, BLACK);
            } else {
                M5.Lcd.setTextColor(TFT_DARKGREY, BLACK);
            }
            M5.Lcd.setCursor(140, 28+i*24);
            if (i != ADJUST_NVS && i != ADJUST_MAG_CALIB) {
                if (adjust_range[i].f == 0)
                    M5.Lcd.printf("%4d", adjust[i]);
                else
                    M5.Lcd.printf("%4.1f", (float)adjust[i]/adjust_range[i].f);
            } else if (i == ADJUST_MAG_CALIB) {
                if (mag_calib_state == 0) {
                    M5.Lcd.printf("Rotate horizontally");
                } else if (mag_calib_state == 1) {
                    M5.Lcd.printf("Turn upside down");
                } else {
                    M5.Lcd.printf("Completed!");
                }
            } else {
                M5.Lcd.printf("A:Load C:Save");
            }
            update = false;
        }
    }

    M5.update();
    delay(1);
}

// The arduino task
void loopTask(void *pvParameters)
{
    setup();

    nvs_handle storage_handle;
    esp_err_t err;
    err = nvs_open("storage", NVS_READWRITE, &storage_handle);
    if (err != ESP_OK) {
        Serial.println("NVS can't be opened");
    } else {
        int32_t v;
        for (int i = 0; i < N_ADJUST - 1; i++) {
            err = nvs_get_i32(storage_handle, var[i], &v);
            if (err != ESP_OK) v = 0;
            adjust[i] = v;
        }
        nvs_close(storage_handle);
    }

    if (M5.BtnA.isPressed() || M5.BtnB.isPressed() || M5.BtnC.isPressed()) {
        trim_mode = true;

        for (int i = 0; i < N_ADJUST; i++) {
            nvs_close(storage_handle);
            M5.Lcd.setTextColor(TFT_DARKGREY, BLACK);
            M5.Lcd.setCursor(30, 28+i*24);
            M5.Lcd.printf(adjust_string[i]);
            M5.Lcd.setCursor(140, 28+i*24);
            if (i == ADJUST_NVS) {
                M5.Lcd.printf("A:Load C:Save");
            } else if (adjust_range[i].f == 0) {
                M5.Lcd.printf("%4d", adjust[i]);
            } else {
                M5.Lcd.printf("%4.1f", (float)adjust[i]/adjust_range[i].f);
            }
        }
    }
        
    for(;;) {
        micros(); //update overflow
        if (trim_mode) {
            loop_conf();
        } else {
            loop();
        }
    }
}

void nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        const esp_partition_t *pa;
        pa = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
        assert(pa && "partition table must have an NVS partition");
        ESP_ERROR_CHECK(esp_partition_erase_range(pa, 0, pa->size));
        // Retry nvs_flash_init
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

extern void imu_task(void *arg);

extern "C" void app_main()
{
    initArduino();

    nvs_init();

    nvs_handle storage_handle;
    esp_err_t err;
    err = nvs_open("storage", NVS_READWRITE, &storage_handle);
    if (err == ESP_OK) {
        int32_t v;
        err = nvs_get_i32(storage_handle, "mx_off", &v);
        if (err == ESP_OK) {
            m5_mag_offset[0] = (float)v;
        }
        err = nvs_get_i32(storage_handle, "my_off", &v);
        if (err == ESP_OK) {
            m5_mag_offset[1] = (float)v;
        }
        err = nvs_get_i32(storage_handle, "mz_off", &v);
        if (err == ESP_OK) {
            m5_mag_offset[2] = (float)v;
        }
        printf("m5_mag_offset loaded %4.0f %4.0f %4.0f\n",
               m5_mag_offset[0], m5_mag_offset[1], m5_mag_offset[2]);
        err = nvs_get_i32(storage_handle, "adj_2", &v);
        if (err == ESP_OK) {
            filter_gain = (float)v/adjust_range[ADJUST_GAIN].f;
        }
        printf("filter_gain loaded %4.1f\n", filter_gain);
        nvs_close(storage_handle);
    }

    att_queue = xQueueCreate(32, 4*sizeof(float));

    xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreate(imu_task, "imu_task", 8192, NULL, 1, NULL);
}
