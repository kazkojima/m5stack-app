/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <M5Stack.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"

#undef F
#include <mavlink_types.h>
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS 1
#define MAVLINK_SEND_UART_BYTES send_tcp_bytes
static void send_tcp_bytes(mavlink_channel_t chan, const uint8_t *buf, uint16_t len);
extern mavlink_system_t mavlink_system;
#include <mavlink.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

const char *networkName = CONFIG_SSID;
const char *networkPswd = CONFIG_SSID_PASSWORD;

//APM server IP address
const char *udpAddress = CONFIG_APM_SERVER_ADDRESS;
const int udpPort = CONFIG_UDP_PORT;
const char *tcpAddress = CONFIG_APM_SERVER_ADDRESS;
const int tcpPort = CONFIG_TELEMETORY_PORT;

//Are we currently connected?
boolean connected = false;
boolean telemetry_connected = false;

//The udp library class
WiFiUDP udp;

//TCP telemetory client
WiFiClient client;

struct __attribute__((packed)) rcpkt {
    uint32_t version;
    uint64_t timestamp_us;
    uint16_t sequence;
    uint16_t pwms[8];
};

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

          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          if(client.connect(tcpAddress, tcpPort))
              telemetry_connected = true;
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

#if CONFIG_THROTTLE_BUTTON
    //M5.Speaker.mute();
#else
    // ??? analogRead makes some click noise on M5 speaker even when
    // it's muted. Looks that the speaker output pin(25) is effected by
    // electorical noise. Zero DAC output to pin 25 reduces that click
    // sound, though there is still small residual noise.
    M5.Speaker.write(0);
#endif

    // LCD display
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setCursor(20, 0);
    M5.Lcd.printf("M5 propo");

    //Connect to the WiFi network
    connectToWiFi(networkName, networkPswd);
}

extern "C" {
    bool trim_mode = false;
    extern float m5_mag_offset[3];
    extern float mx_raw_min, mx_raw_max;
    extern float my_raw_min, my_raw_max;
    extern float mz_raw_min, mz_raw_max;

    xQueueHandle att_queue = NULL;
    void imu_task(void *arg);
}

boolean telemetry_yaw_ok = false;
float telemetry_yaw;

uint8_t target_sysid;
uint8_t target_compid;
mavlink_system_t mavlink_system = { 20, 99 };
boolean request_sent = false;

static void send_tcp_bytes(mavlink_channel_t chan, const uint8_t *buf,
                           uint16_t len)
{
    client.write(buf, len);
}

void check_telemetry(void)
{
    if (!telemetry_connected)
        return;
    for(;;) {
        if (client.available() > 0) {
            uint8_t c = client.read();
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
                switch(msg.msgid)
                    {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        target_sysid = msg.sysid;
                        target_compid = msg.compid;
                        if (!request_sent) {
                            const uint8_t stream_id = MAV_DATA_STREAM_EXTRA1;
                            mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
                                                                 target_sysid,
                                                                 target_compid,
                                                                 stream_id,
                                                                 10,
                                                                 1);
                            request_sent = true;
                        }
                        break;
                    case MAVLINK_MSG_ID_ATTITUDE:
                        //Serial.println("ATTITUDE");
                        mavlink_attitude_t attitude;
                        mavlink_msg_attitude_decode(&msg, &attitude);
                        telemetry_yaw = attitude.yaw;
                        telemetry_yaw_ok = true;
                        //Serial.println(telemetry_yaw);
                        break;
                    default:
                        break;
                      }
            }
        } else
            break;
    }
}

static uint16_t pwm_sat(uint16_t pwm)
{
    if (pwm > 1900)
        return 1900;
    else if (pwm < 1100)
        return 1100;
    return pwm;
}

#if CONFIG_THROTTLE_BUTTON
const float throttle_hover = (CONFIG_THROTTLE_HOVER - 1100) / 800.0;
#endif
float throttle = 0.0; // % of throttle
uint16_t rcpkt_count;
int count;

#if CONFIG_DISPLAY_TYPE_BAR
const uint8_t xo = 28;
const uint8_t yo = 60;
const uint8_t bw = 160;
#elif CONFIG_DISPLAY_TYPE_MARK
int32_t prev_cx, prev_cy, prev_mx, prev_my;
#endif

enum adjust {
    CH1_OFFSET = 0,
    CH2_OFFSET,
    CH3_OFFSET,
    CH4_OFFSET,
    YAW_SENSE,
    YAW_FIX,
    MAG_CALIB,
    ADJUST_NVS
};
#define N_ADJUST 8
int adjust_index;
int16_t adjust[N_ADJUST];
const struct { int16_t min; int16_t max; } adjust_range[N_ADJUST] = { 
    { -200, +200 },
    { -200, +200 },
    { -200, +200 },
    { -200, +200 },
    { 0, 15 },
    { -90, 90 },
    { 0, 0 },
    { 0, 0 },
};
const char *adjust_string[N_ADJUST] = {
    "Roll offset",
    "Pitch offset",
    "Throttle offset",
    "Yaw offset",
    "Yaw sensitivity",
    "Yaw misalign",
    "Mag calibration",
    "Load/Save",
};
    
static inline float curve(float x)
{
#if 1
    float v;
    if (x < 0)
        v = - x*x;
    else
        v = x*x;
    x = 2*v;
#endif
    x += 0.5;
    if (x < 0) return 0.0;
    else if (x > 1) return 1.0;
    return x;
}

// The loop routine runs over and over again forever
void loop() {

    check_telemetry();

    float att[4];
    if (xQueueReceive(att_queue, &att[0], 0) == pdTRUE) {
        float yawerr = 0;
        if (telemetry_yaw_ok) {
            float yawfix = adjust[YAW_FIX] / 57.29578f;
            float tel_n = cosf(telemetry_yaw - yawfix);
            float tel_e = sinf(telemetry_yaw - yawfix);
            float r = sqrtf(att[2]*att[2]+ att[3]*att[3]);
            //Serial.println("n "+String(tel_n)+" e "+String(tel_e));
            //Serial.println("N "+String(att[2]/r)+" E "+String(att[3]/r));
            yawerr = -(tel_e * (att[2]/r) - tel_n * (att[3]/r));
            if (yawerr > 1.0)
                yawerr = 1.0;
            else if (yawerr < -1.0)
                yawerr = -1.0;
            yawerr *= adjust[YAW_SENSE] / 100.0f;
            //Serial.println("E "+String(yawerr));
        }
   
        if ((count % 10) == 0) {
#if CONFIG_DISPLAY_TYPE_BAR
            uint8_t v;
            // Display roll bar
            v = (uint8_t)(curve(att[0])*bw);
            M5.Lcd.fillRect(xo, 40, v, 16, CYAN);
            M5.Lcd.fillRect(xo+v, 40, bw-v, 16, TFT_DARKGREY);
            // Display pitch bar
            v = (uint8_t)(curve(att[1])*bw);
            M5.Lcd.fillRect(100, yo, 24, bw-v, TFT_DARKGREY);
            M5.Lcd.fillRect(100, yo+bw-v, 24, v, GREEN);
            // Display throttle bar
            v = (uint8_t)(throttle*bw);
            M5.Lcd.fillRect(280, yo, 24, bw-v, TFT_DARKGREY);
            M5.Lcd.fillRect(280, yo+bw-v, 24, v, ORANGE);
            // Display yaw bar. Test only.
            v = (uint8_t)((yawerr+0.5)*bw);
            M5.Lcd.fillRect(xo, 224, v, 16, YELLOW);
            M5.Lcd.fillRect(xo+v, 224, bw-v, 16, TFT_DARKGREY);
#elif CONFIG_DISPLAY_TYPE_MARK
            int32_t cx, cy;
            cx = (int32_t)(240*att[0]);
            cy = (int32_t)(240*att[1]);
            cx = 160 + cx;
            cy = 120 + cy;
            if (cx < 0)
                cx = 0;
            if (cy < 0)
                cy = 0;
#if CONFIG_DISPLAY_COMPASS
            int32_t mx, my;
            mx = (int32_t)(120*att[3]); // East
            my = (int32_t)(120*att[2]); // North
            mx = 160 + mx;
            my = 120 - my;
            if (mx < 0)
                mx = 0;
            if (my < 0)
                my = 0;
            M5.Lcd.fillRect(prev_mx, prev_my, 16, 16, BLACK);
#endif
            M5.Lcd.fillRect(prev_cx, prev_cy, 16, 16, BLACK);
#if CONFIG_DISPLAY_COMPASS
            M5.Lcd.setTextColor(CYAN, BLACK);
            M5.Lcd.setCursor(mx, my);
            M5.Lcd.print("N");
            prev_mx = mx; prev_my = my;
#endif
            M5.Lcd.setTextColor(YELLOW, BLACK);
            M5.Lcd.setCursor(cx, cy);
            M5.Lcd.print("x");
            prev_cx = cx; prev_cy = cy;

            // Display throttle bar
            uint8_t h = (uint8_t)(throttle*240);
            M5.Lcd.fillRect(320-16, 0, 15, 240-h, BLACK);
            M5.Lcd.fillRect(320-16, 240-h, 15, 240, ORANGE);
#endif
        }

        float pwm_thr, pwm_pitch, pwm_roll, pwm_yaw;
            pwm_roll = pwm_sat((int16_t)(curve(att[0])*800) + 1100
                               + adjust[CH1_OFFSET]);
            pwm_pitch = pwm_sat((int16_t)(curve(att[1])*800) + 1100
                                + adjust[CH2_OFFSET]);
            pwm_yaw = pwm_sat((int16_t)(yawerr*800) + 1500
                              + adjust[CH4_OFFSET]);
#if CONFIG_THROTTLE_BUTTON
        if(M5.BtnA.isPressed()) {
            throttle -= 0.002;
            if (throttle < 0)
                throttle = 0;
        }
        if (M5.BtnC.isPressed()) {
            throttle += 0.002;
            if (throttle > 100.0)
                throttle = 100.0;
        }
        if (M5.BtnB.isPressed()) {
            if (throttle < throttle_hover) {
                throttle += 0.01;
                if (throttle > throttle_hover)
                    throttle = throttle_hover;
            } else if (throttle > throttle_hover) {
                throttle -= 0.01;
                if (throttle < throttle_hover)
                    throttle = throttle_hover;
            }
        }
#else
        float adc = analogRead(35) / 4095.0;
        adc = 1 - adc;
        // make dead band <10% and >90%
        if (adc < 0.1)
            adc = 0.1;
        if (adc > 0.9)
            adc = 0.9;
        adc = (adc - 0.1) / 0.8;
        throttle = 0.9*throttle + 0.1*adc;
#endif
        pwm_thr = pwm_sat((uint16_t)(throttle*800) + 1100 + adjust[CH3_OFFSET]);

        if (connected) {
            struct rcpkt pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.version = 2;
            pkt.sequence = rcpkt_count++;
            pkt.timestamp_us = ((uint64_t)1000) * millis();
            pkt.pwms[0] = pwm_roll;
            pkt.pwms[1] = pwm_pitch;
            pkt.pwms[2] = pwm_thr;
            pkt.pwms[3] = pwm_yaw;
            for(int i=4; i<8; i++)
                pkt.pwms[i] = 1500;
            udp.beginPacket(udpAddress,udpPort);
            udp.write((const uint8_t *)&pkt, sizeof(pkt));
            udp.endPacket();
        }

        if (!telemetry_connected && (count % 1000) == 0) {
          if(client.connect(tcpAddress, tcpPort))
              telemetry_connected = true;
        }

        count++;
    }

    M5.update();
    delay(1);
}

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

const char *var[] = {
    "adj_0", "adj_1", "adj_2", "adj_3", "adj_4", "adj_5", "adj_6", };

// Alternative loop for configuration mode
void loop_conf() {
    static bool update = true;
    if (M5.BtnB.wasPressed()) {
        adjust_index = (adjust_index + 1) % N_ADJUST;
        update = true;
    }
    if (adjust_index != ADJUST_NVS && adjust_index != MAG_CALIB) {
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
    } else if (adjust_index == MAG_CALIB) {
        if (M5.BtnA.wasPressed()) {
            // restert calib
            mx_raw_min = my_raw_min = mz_raw_min = MAG_LIM;
            mx_raw_max = my_raw_max = mz_raw_max = -MAG_LIM;
            mag_calib_state = 0;
            update = true;
        }
        update = (update || guess_mag_offset());
    } else {
        nvs_handle storage_handle;
        esp_err_t err;
        if (M5.BtnA.wasPressed()) {
            // Load
            err = nvs_open("storage", NVS_READWRITE, &storage_handle);
            if (err != ESP_OK) {
                Serial.println("NVS can't be opened");
            } else {
                int32_t v;
                for (int i = 0; i < N_ADJUST - 2; i++) {
                    err = nvs_get_i32(storage_handle, var[i], &v);
                    if (err != ESP_OK) v = 0;
                    adjust[i] = v;
                }
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
            if (i != ADJUST_NVS && i != MAG_CALIB) {
                M5.Lcd.printf("%4d", adjust[i]);
            } else if (i == MAG_CALIB) {
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
    }

    if (M5.BtnA.isPressed() || M5.BtnB.isPressed() || M5.BtnC.isPressed()) {
        trim_mode = true;

        for (int i = 0; i < N_ADJUST; i++) {
            nvs_close(storage_handle);
            M5.Lcd.setTextColor(TFT_DARKGREY, BLACK);
            M5.Lcd.setCursor(30, 28+i*24);
            M5.Lcd.printf(adjust_string[i]);
            M5.Lcd.setCursor(140, 28+i*24);
            if (i != ADJUST_NVS) {
                M5.Lcd.printf("%4d", adjust[i]);
            } else {
                M5.Lcd.printf("A:Load C:Save");
            }
        }
    } else {
#if CONFIG_DISPLAY_TYPE_BAR
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.setCursor(40,28);
        M5.Lcd.printf("CH1: ROLL");
        M5.Lcd.setCursor(30,130);
        M5.Lcd.printf("CH2: PITCH");
        M5.Lcd.setCursor(40,212);
        M5.Lcd.printf("CH4: YAW");
        M5.Lcd.setCursor(196,130);
        M5.Lcd.printf("CH3: THROTTLE");
#endif
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
    }

    att_queue = xQueueCreate(32, 4*sizeof(float));

    xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreate(imu_task, "imu_task", 2048, NULL, 1, NULL);
}
