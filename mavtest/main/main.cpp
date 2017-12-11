/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <M5Stack.h>
#include <WiFi.h>
#undef F
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS 1
#define MAVLINK_SEND_UART_BYTES send_tcp_bytes
#include <mavlink.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define NO_DYSPLAY 0
#define DISPLAY_BAR 1
#define DISPLAY_MARK 2

const char *networkName = CONFIG_SSID;
const char *networkPswd = CONFIG_SSID_PASSWORD;

//IP address for telemetory:
// either use the ip address of the server or
// a network broadcast address
const char *tcpAddress = CONFIG_APM_SERVER_ADDRESS;
const int tcpPort = CONFIG_TELEMETORY_PORT;

//Are we currently connected?
boolean connected = false;

// UDP telemetory socket
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
          M5.Lcd.setTextColor(GREEN ,BLACK);
          M5.Lcd.setCursor(20,0);
          M5.Lcd.print("WiFi connected! IP address: ");
          M5.Lcd.print(WiFi.localIP());

          client.connect(tcpAddress, tcpPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");

          // Start device display with ID of sensor
          M5.Lcd.fillRect(20, 0, 300, 16, BLACK);
          M5.Lcd.setTextColor(RED ,BLACK);
          M5.Lcd.setCursor(20,0);
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

    // LCD display
    M5.Lcd.setTextColor(GREEN ,BLACK);
    M5.Lcd.setCursor(20,0);
    M5.Lcd.printf("mavlink test");

    //Connect to the WiFi network
    connectToWiFi(networkName, networkPswd);
}

extern "C" {
}

uint8_t target_sysid;
uint8_t target_compid;
mavlink_system_t mavlink_system = { 20, 99 };
boolean request_sent = false;

static void send_tcp_bytes(mavlink_channel_t chan, const uint8_t *buf,
                           uint16_t len)
{
    client.write(buf, len);
}

// The loop routine runs over and over again forever
void loop() {

    if (connected) {
        if (client.available() > 0) {
            uint8_t c = client.read();
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
                switch(msg.msgid)
                    {
                    case MAVLINK_MSG_ID_HEARTBEAT:
#if 0
                        Serial.print("system:");
                        Serial.print(msg.sysid);
                        Serial.print(" component:");
                        Serial.println(msg.compid);
                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                        M5.Lcd.fillRect(0, 40, 320, 200, BLACK);
                        M5.Lcd.setTextColor(WHITE ,BLACK);
                        M5.Lcd.setCursor(10, 40);
                        M5.Lcd.print("HERATBEAT");
                        M5.Lcd.setCursor(20, 60);
                        M5.Lcd.print("version:");
                        M5.Lcd.print(heartbeat.mavlink_version);
                        M5.Lcd.setCursor(20, 80);
                        M5.Lcd.print("status:");
                        M5.Lcd.print(heartbeat.system_status);
                        M5.Lcd.setCursor(20, 100);
                        M5.Lcd.print("type:");
                        M5.Lcd.print(heartbeat.type);
                        M5.Lcd.setCursor(20, 120);
                        M5.Lcd.print("autopilot:");
                        M5.Lcd.print(heartbeat.autopilot);
                        M5.Lcd.setCursor(20, 140);
                        M5.Lcd.print("base_mode:");
                        M5.Lcd.print(heartbeat.base_mode);
                        M5.Lcd.setCursor(20, 160);
                        M5.Lcd.print("custum_mode:");
                        M5.Lcd.print(heartbeat.custom_mode);
#else
                        target_sysid = msg.sysid;
                        target_compid = msg.compid;
                        if (!request_sent) {
                            const uint8_t stream_id = MAV_DATA_STREAM_EXTRA1;
                            mavlink_msg_request_data_stream_send((mavlink_channel_t)0,
                                                                 target_sysid,
                                                                 target_compid,
                                                                 stream_id,
                                                                 4,
                                                                 1);
                            request_sent = true;
                        }
#endif
                        break;
                    case MAVLINK_MSG_ID_ATTITUDE:
                        //Serial.println("ATTITUDE");
                        mavlink_attitude_t attitude;
                        mavlink_msg_attitude_decode(&msg, &attitude);
                        M5.Lcd.fillRect(0, 40, 320, 200, BLACK);
                        M5.Lcd.setTextColor(WHITE ,BLACK);
                        M5.Lcd.setCursor(10, 40);
                        M5.Lcd.print("MAVLINK ATTITUDE");
                        M5.Lcd.setTextColor(YELLOW ,BLACK);
                        M5.Lcd.setCursor(20, 60);
                        M5.Lcd.print("roll:");
                        M5.Lcd.print(attitude.roll);
                        M5.Lcd.setCursor(20, 80);
                        M5.Lcd.print("pitch:");
                        M5.Lcd.print(attitude.pitch);
                        M5.Lcd.setCursor(20, 100);
                        M5.Lcd.print("yaw:");
                        M5.Lcd.print(attitude.yaw);
                        M5.Lcd.setCursor(20, 120);
                        M5.Lcd.print("rollspeed:");
                        M5.Lcd.print(attitude.rollspeed);
                        M5.Lcd.setCursor(20, 140);
                        M5.Lcd.print("pitchspeed:");
                        M5.Lcd.print(attitude.pitchspeed);
                        M5.Lcd.setCursor(20, 160);
                        M5.Lcd.print("yawspeed:");
                        M5.Lcd.print(attitude.yawspeed);
                        M5.Lcd.setCursor(20, 180);
                        M5.Lcd.print("time_boot_ms:");
                        M5.Lcd.print(attitude.time_boot_ms);
                    default:
                        break;
                    }
            }
        }
    }

    M5.update();
    delay(1);
}

// The arduino task
void loopTask(void *pvParameters)
{
    setup();
    for(;;) {
        micros(); //update overflow
        loop();
    }
}

extern "C" void app_main()
{
    initArduino();

    xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
}
