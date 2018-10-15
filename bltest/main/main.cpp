/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <M5Stack.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

extern "C" {
    extern xQueueHandle key_queue;
    extern xQueueHandle pt_queue;
    struct mouse { uint8_t dir; int8_t dat; };
};

void setup(){

    // Initialize the M5Stack object
    M5.begin();

    M5.Speaker.write(0);

    // LCD display
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(20, 0);
    M5.Lcd.printf("M5 bluetooth test");
    M5.Lcd.setTextColor(WHITE, BLACK);
}

void loop()
{
    uint8_t ch;
    struct mouse m;
    static int pos = 0;
    static int xpos = 0, ypos = 0;
    if (xQueueReceive(key_queue, &ch, 0) == pdTRUE) {
        M5.Lcd.setCursor(20+16*pos, 20);
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.printf("%c", ch);
        pos = (pos + 1)%16;
    }
    if (xQueueReceive(pt_queue, &m, 0) == pdTRUE) {
        M5.Lcd.setTextSize(1);
        //M5.Lcd.setCursor(20, 40);
        //M5.Lcd.printf("%02x %02x", m.dir, m.dat);
        if (m.dir == 0 && m.dat != 0) {
            if (m.dat + xpos < 800 && -800 < m.dat + xpos) {
                M5.Lcd.setCursor(160+xpos/8, 120+ypos/8);
                M5.Lcd.printf(" ");
                xpos += m.dat;
                M5.Lcd.setCursor(160+xpos/8, 120+ypos/8);
                M5.Lcd.setTextColor(YELLOW, BLACK);
                M5.Lcd.printf("+");
            }
        }
        if (m.dir == 1 && m.dat != 0) {
            if (m.dat + ypos < 600 && -600 < m.dat + ypos) {
                M5.Lcd.setCursor(160+xpos/8, 120+ypos/8);
                M5.Lcd.printf(" ");
                ypos += m.dat;
                M5.Lcd.setCursor(160+xpos/8, 120+ypos/8);
                M5.Lcd.setTextColor(YELLOW, BLACK);
                M5.Lcd.printf("+");
            }
        }
        M5.Lcd.setTextSize(2);
    }
    M5.update();
    delay(1);
}

// The arduino task
void loopTask(void *pvParameters)
{
    setup();

    while(1) {
        loop ();
    }
}

extern "C" void m5_arduino_main()
{
    initArduino();

    xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
}
 

