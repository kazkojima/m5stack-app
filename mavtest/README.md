# mavtest application

A tiny application to test a few mavlink functions. Based on M5Stack-IDF.

You need to link or copy [M5Stack-IDF](https://github.com/m5stack/M5Stack-IDF)/components directory:

```shell
ln -s <path_to_M5Stack-IDF_components_directory> ./components
```

and add MAVLink [c_library_v2](https://github.com/mavlink/c_library_v2) as a new component:

```shell
mkdir <path_to_M5Stack-IDF_components_directory>/mavlink
mv c_library_v2 <path_to_M5Stack-IDF_components_directory>/mavlink/v2.0
cat << EOS > <path_to_M5Stack-IDF_components_directory>/mavlink/component.mk
#
# Main Makefile. This is basically the same as a component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_ADD_INCLUDEDIRS := v2.0 v2.0/common
EOS
```

The tcp telemetry port of ArduCopter can be assigned with its command line argument like as "-C tcp:192.168.11.1:5900".  These address(APM_SERVER_ADDRESS) and port(TELEMETORY_PORT) of telemetry can be configured with "make menuconfig".
