#!/bin/bash

release_foldername="DroneBridge_ESP32_vXX"
release_name_zip="DroneBridge_ESP32_vXX.zip"

mkdir "$release_foldername"
mkdir build
cp flashing_instructions.txt "$release_foldername"

rm -rf build
idf.py fullclean
cp sdkconfig_esp32 sdkconfig
idf.py build
mkdir "$release_foldername/esp32"
cp build/flash_args "$release_foldername/esp32/flash_args.txt"
cp build/db_esp32.bin "$release_foldername/esp32"
cp build/bootloader/bootloader.bin "$release_foldername/esp32"
cp build/www.bin "$release_foldername/esp32"
cp build/partition_table/partition-table.bin "$release_foldername/esp32"

rm -rf build
idf.py fullclean
cp sdkconfig_s2 sdkconfig
idf.py build
mkdir "$release_foldername/esp32s2"
cp build/flash_args "$release_foldername/esp32s2/flash_args.txt"
cp build/db_esp32.bin "$release_foldername/esp32s2"
cp build/bootloader/bootloader.bin "$release_foldername/esp32s2"
cp build/www.bin "$release_foldername/esp32s2"
cp build/partition_table/partition-table.bin "$release_foldername/esp32s2"

rm -rf build
idf.py fullclean
cp sdkconfig_s3 sdkconfig
idf.py build
mkdir "$release_foldername/esp32s3"
cp build/flash_args "$release_foldername/esp32s3/flash_args.txt"
cp build/db_esp32.bin "$release_foldername/esp32s3"
cp build/bootloader/bootloader.bin "$release_foldername/esp32s3"
cp build/www.bin "$release_foldername/esp32s3"
cp build/partition_table/partition-table.bin "$release_foldername/esp32s3"

rm -rf build
idf.py fullclean
cp sdkconfig_c3 sdkconfig
idf.py build
mkdir "$release_foldername/esp32c3"
cp build/flash_args "$release_foldername/esp32c3/flash_args.txt"
cp build/db_esp32.bin "$release_foldername/esp32c3"
cp build/bootloader/bootloader.bin "$release_foldername/esp32c3"
cp build/www.bin "$release_foldername/esp32c3"
cp build/partition_table/partition-table.bin "$release_foldername/esp32c3"

rm -rf build
idf.py fullclean
cp sdkconfig_c3_serial_via_JTAG sdkconfig
idf.py build
mkdir "$release_foldername/esp32c3_USBSerial"
cp build/flash_args "$release_foldername/esp32c3_USBSerial/flash_args.txt"
cp build/db_esp32.bin "$release_foldername/esp32c3_USBSerial"
cp build/bootloader/bootloader.bin "$release_foldername/esp32c3_USBSerial"
cp build/www.bin "$release_foldername/esp32c3_USBSerial"
cp build/partition_table/partition-table.bin "$release_foldername/esp32c3_USBSerial"

if [ -f "$release_name_zip" ]; then
   rm "$release_name_zip"
fi
zip -r "$release_name_zip" "$release_foldername"

rm -rf "$release_foldername"
