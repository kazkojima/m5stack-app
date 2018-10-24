# An example i2s input application

A tiny and simple I2S input sample. Tested with ADMP411 mic and AK5720 audio adc.

You need to link or copy [M5Stack-IDF](https://github.com/m5stack/M5Stack-IDF)/components directory:

```shell
ln -s <path_to_M5Stack-IDF_components_directory> ./components
```

This sample sends S24_LE format signal to the host's UDP port. One can replay the result with

```shell
socat udp-listen:5990,fork,reuseaddr - | aplay -f S24_LE -r 8000
```
or, for the stereo case,
```shell
socat udp-listen:5990,fork,reuseaddr - | aplay -f S24_LE -r 8000 -c 2
```
on the host. The IP address/UDP port and AP SSID/password are configurable with 'make menuconfig'.

Also I2S_DISPLAY_FFT is configurable at menu. If set, M5Stack will display simple FFT results on LCD. This sample uses GPFFT to do FFT. See main/readme.GPFFT.txt for details and the license of GPFFT.
