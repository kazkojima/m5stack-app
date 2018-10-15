# A bluetooth host example using btstack

[Quick instructions]

0) Clone this to _SOMEWHERE_
1) Clone https://github.com/bluekitchen/btstack.git
2) cd btstack/port/esp32
3) IDF_PATH=_absolute_path_of_SOMEWHERE_ ./integrate_btstack.py

The Step 3 creates BTstack component in _SOMEWHERE_/componemts/.

Assume that https://github.com/m5stack/M5Stack-IDF.git is cloned at /_A_/
and https://github.com/espressif/arduino-esp32.git is cloned at /_B_/.

4) cd _SOMEWHERE_ & mkdir components
6) ln -s /_A_/M5Stack-IDF/components/m5stack componemts/m5stack
5) ln -s /_B_/arduino-esp32 componemts/arduino
7) cp /_A_/M5Stack-IDF/sdkconfig .
8) make menuconfig
   Enable BT with 'Component config' ---> 'Bluetooth' ---> [*] Bluetooth
8) make

[Remark]

Notice that M5Stack-IDF has the original arduino component components/arduino which is arduino-esp32 + BLE submodule. Unfortunately, that BLE library mismatches with the latest esp-idf, ATM(14/Oct/2018), and it results a build failure. Perhaps another workaround is to filter that library out with something like the temporally patch below at M5Stack-IDF/components/arduino/.

```
diff --git a/component.mk b/component.mk
index 15d50c0..868bae5 100644
--- a/component.mk
+++ b/component.mk
@@ -1,4 +1,5 @@
-ARDUINO_CORE_LIBS := $(patsubst $(COMPONENT_PATH)/%,%,$(sort $(dir $(wildcard $(COMPONENT_PATH)/libraries/*/*/))))
+ARDUINO_BLE_LIBS := libraries/BLE/src/
+ARDUINO_CORE_LIBS := $(filter-out $(ARDUINO_BLE_LIBS),$(patsubst $(COMPONENT_PATH)/%,%,$(sort $(dir $(wildcard $(COMPONENT_PATH)/libraries/*/*/)))))
 
 COMPONENT_ADD_INCLUDEDIRS := cores/esp32 variants/esp32 $(ARDUINO_CORE_LIBS)
 COMPONENT_PRIV_INCLUDEDIRS := cores/esp32/libb64
```

[A bit of background]

esp-idf has the component mechanism which can utilize the external library modules. It searches some fixed directories so as to find the sub-directories which have component.mk files and interprets contents of them as the addends for makefile.

In this example, it finds arduino, m5stack and btstack as directories which have component.mk and compiles/links the files in these directories according to the each component.mk. The above instruction 1) to 6) simply set these extra libraries at components/ directory. After these instructions, one will get the tree like:


```
.
├── Makefile
├── README.md
├── components
│   ├── arduino -> /B/arduino-esp32
│   ├── btstack
│   └── m5stack -> /A/M5Stack-IDF/components/m5stack
├── main
│   ├── component.mk
│   ├── hid_host_demo.c
│   └── main.cpp
└── sdkconfig
```

The arduino loop is executed as a freertos task pinned to one core of ESP32. Thus you can use arduino elements from that loop and can use freertos stuff at the same time.

btstack defines app_main() function which calls btstack_main() in hid_host_demo.c. This btstack_main() calls m5_arduino_main() function at its end and m5_arduino_main() in main.cpp starts the arduino loop task.

hid_host_demo.c is the almost same with the original one of btstack/port/esp32/example/hid_host_demo but has the above m5_arduino_main() call and freertos queue definitions used so that arduino/non-arduino tasks can communicate. I add a very simple handling for mouse data and a part of btstack/port/esp32/example/gap_inquery example to scan BT, too. Currently, the latter picks up the first HID client found. Although clearly wrong when there exists multiple candidates, it removes the hard coding of BT address.
