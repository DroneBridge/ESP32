# Execute in an esp-idf enabled PowerShell
# This script will create a combined zip file containing binaries for all supported esp32 boards

$release_foldername = "DroneBridge_ESP32_vXX"
$release_name_zip = "DroneBridge_ESP32_vXX.zip"

mkdir $release_foldername
cp .\flashing_instructions.txt $release_foldername

rm -Recurse .\build
idf.py set-target esp32 build
mkdir $release_foldername\esp32
cp .\build\flash_args $release_foldername\esp32
cp .\build\db_esp32.bin $release_foldername\esp32
cp .\build\bootloader\bootloader.bin $release_foldername\esp32
cp .\build\www.bin $release_foldername\esp32
cp .\build\partition_table\partition-table.bin $release_foldername\esp32

rm -Recurse .\build
idf.py set-target esp32s2 build
mkdir $release_foldername\esp32s2
cp .\build\flash_args $release_foldername\esp32s2
cp .\build\db_esp32.bin $release_foldername\esp32s2
cp .\build\bootloader\bootloader.bin $release_foldername\esp32s2
cp .\build\www.bin $release_foldername\esp32s2
cp .\build\partition_table\partition-table.bin $release_foldername\esp32s2

rm -Recurse .\build
idf.py set-target esp32s3 build
mkdir $release_foldername\esp32s3
cp .\build\flash_args $release_foldername\esp32s3
cp .\build\db_esp32.bin $release_foldername\esp32s3
cp .\build\bootloader\bootloader.bin $release_foldername\esp32s3
cp .\build\www.bin $release_foldername\esp32s3
cp .\build\partition_table\partition-table.bin $release_foldername\esp32s3

rm -Recurse .\build
idf.py set-target esp32c3 build
mkdir $release_foldername\esp32c3
cp .\build\flash_args $release_foldername\esp32c3
cp .\build\db_esp32.bin $release_foldername\esp32c3
cp .\build\bootloader\bootloader.bin $release_foldername\esp32c3
cp .\build\www.bin $release_foldername\esp32c3
cp .\build\partition_table\partition-table.bin $release_foldername\esp32c3

if (Test-Path $release_name_zip) {
   Remove-Item $release_name_zip -verbose
}
Compress-Archive -Path $release_foldername -DestinationPath $release_name_zip

rm -R $release_foldername