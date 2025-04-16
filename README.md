# murmecha-cam


# menuconfig adjustments

- enable -fno-rtti in menuconf (error when linking: libopencv_imgcodecs.a(loadsave.cpp.obj):(.data.rel.ro._ZTIN2cv12_GLOBAL__N_116ByteStreamBufferE+0x8): undefined reference to _ZTISt15basic_streambufIcSt11char_traitsIcEE')
- enable PSRAM -> set octal
- enable partition
- disable watchdog
- enable master log level (to surpress logs when recording) 

# other

lib from https://github.com/joachimBurket/esp32-opencv

menuconfig adjustments for ESP32-S3-WROOM-1-N16R8: https://github.com/joachimBurket/esp32-opencv/pull/23

# degbugging

https://docs.espressif.com/projects/esp-idf/en/v5.0/esp32s3/api-guides/jtag-debugging/index.html

clion extension: https://developer.espressif.com/blog/clion/

debug with extension https://yunyizhi.github.io/ESP-IDF-for-Clion/debug.html#z9palmb_11

make sure to use the JTAG USB port so that the device shows up as JTAG controller https://esp32.com//viewtopic.php?t=38722

install driver (for linux) 

https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-guides/jtag-debugging/configure-ft2232h-jtag.html

- add user to plugdev `sudo usermod -a max -G plugdev`
- `cd /etc/udev/rules.d`
- `wget https://raw.githubusercontent.com/espressif/openocd-esp32/refs/heads/master/contrib/60-openocd.rules`