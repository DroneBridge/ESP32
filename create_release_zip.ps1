# Execute in an esp-idf enabled PowerShell
# This script will create a combined zip file containing binaries for all supported esp32 boards

$release_foldername = "DroneBridge_ESP32_v2_2_2_RC1"
$release_name_zip = "DroneBridge_ESP32_v2_2_2_RC1.zip"

mkdir $release_foldername
mkdir build
cp .\flashing_instructions.txt $release_foldername

function BuildAndCopy($config, $folder, $target) {
    rm -Recurse -Force .\build
    idf.py fullclean
    idf.py set-target $target
    cp .\$config .\sdkconfig
    idf.py build
    mkdir $release_foldername\$folder
    cp .\build\flash_args $release_foldername\$folder\flash_args.txt
    cp .\build\db_esp32.bin $release_foldername\$folder
    cp .\build\bootloader\bootloader.bin $release_foldername\$folder
    cp .\build\www.bin $release_foldername\$folder
    cp .\build\partition_table\partition-table.bin $release_foldername\$folder
}

# ESP32
BuildAndCopy "sdkconfig_esp32" "esp32" "esp32"
# BuildAndCopy "sdkconfig_esp32_noUARTConsole" "esp32_noUARTConsole" # Build issue - ESP-NOW wants a console for debugging

# ESP32-S2
BuildAndCopy "sdkconfig_s2" "esp32s2" "esp32s2"
BuildAndCopy "sdkconfig_s2_noUARTConsole" "esp32s2_noUARTConsole" "esp32s2"

# ESP32-S3
BuildAndCopy "sdkconfig_s3" "esp32s3" "esp32s3"
BuildAndCopy "sdkconfig_s3_noUARTConsole" "esp32s3_noUARTConsole" "esp32s3"
BuildAndCopy "sdkconfig_s3_serial_via_JTAG" "esp32s3_USBSerial" "esp32s3"

# ESP32-C3
BuildAndCopy "sdkconfig_c3" "esp32c3" "esp32c3"
BuildAndCopy "sdkconfig_c3_official" "esp32c3_official" "esp32c3"
BuildAndCopy "sdkconfig_c3_serial_via_JTAG" "esp32c3_USBSerial" "esp32c3"
BuildAndCopy "sdkconfig_c3_noUARTConsole" "esp32c3_noUARTConsole" "esp32c3"

# ESP32-C6
BuildAndCopy "sdkconfig_c6" "esp32c6" "esp32c6"
BuildAndCopy "sdkconfig_c6_official" "esp32c6_official" "esp32c6"
BuildAndCopy "sdkconfig_c6_official_serial_via_JTAG" "esp32c6_official_serial_via_JTAG" "esp32c6"
BuildAndCopy "sdkconfig_c6_official_noUARTConsole" "esp32c6_official_noUARTConsole" "esp32c6"
BuildAndCopy "sdkconfig_c6_serial_via_JTAG" "esp32c6_USBSerial" "esp32c6"
BuildAndCopy "sdkconfig_c6_noUARTConsole" "esp32c6_noUARTConsole" "esp32c6"

if (Test-Path $release_name_zip) {
    Remove-Item $release_name_zip -Verbose
}
Compress-Archive -Path $release_foldername -DestinationPath $release_name_zip

rm -Recurse -Force $release_foldername
