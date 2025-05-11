#!/bin/bash

# Execute in an esp-idf enabled shell
# This script will create a combined zip file containing binaries for all supported esp32 boards

release_foldername="DroneBridge_ESP32_v2_1_nightly"
release_name_zip="DroneBridge_ESP32_v2_1_nightly.zip"

mkdir -p $release_foldername
mkdir -p build
cp ./flashing_instructions.txt $release_foldername

# Function to build and copy binaries for different configurations
build_and_copy() {
    config=$1
    folder=$2

    rm -rf ./build
	cp $config sdkconfig
    idf.py fullclean
    cp $config sdkconfig
    idf.py build
    mkdir -p $release_foldername/$folder
    cp ./build/flash_args $release_foldername/$folder/flash_args.txt
    cp ./build/db_esp32.bin $release_foldername/$folder
    cp ./build/bootloader/bootloader.bin $release_foldername/$folder
    cp ./build/www.bin $release_foldername/$folder
    cp ./build/partition_table/partition-table.bin $release_foldername/$folder
}

# ESP32
build_and_copy sdkconfig_esp32 esp32
# build_and_copy sdkconfig_esp32_noUARTConsole esp32_noUARTConsole # Build issue - ESP-NOW wants a console for debugging

# ESP32-S2
build_and_copy sdkconfig_s2 esp32s2
build_and_copy sdkconfig_s2_noUARTConsole esp32s2_noUARTConsole

# ESP32-S3
build_and_copy sdkconfig_s3 esp32s3
build_and_copy sdkconfig_s3_noUARTConsole esp32s3_noUARTConsole
build_and_copy sdkconfig_s3_serial_via_JTAG esp32s3_USBSerial

# ESP32-C3
build_and_copy sdkconfig_c3 esp32c3
build_and_copy sdkconfig_c3_official esp32c3_official
build_and_copy sdkconfig_c3_serial_via_JTAG esp32c3_USBSerial
build_and_copy sdkconfig_c3_noUARTConsole esp32c3_noUARTConsole

# ESP32-C6
build_and_copy sdkconfig_c6 esp32c6
build_and_copy sdkconfig_c6_official esp32c6_official
build_and_copy sdkconfig_c6_official_noUARTConsole esp32c6_official_noUARTConsole
build_and_copy sdkconfig_c6_official_serial_via_JTAG esp32c6_official_USBSerial
build_and_copy sdkconfig_c6_serial_via_JTAG esp32c6_USBSerial
build_and_copy sdkconfig_c6_noUARTConsole esp32c6_noUARTConsole

# Create the zip file
if [ -f $release_name_zip ]; then
    rm -v $release_name_zip
fi
zip -r $release_name_zip $release_foldername

# Clean up
# rm -rf $release_foldername
