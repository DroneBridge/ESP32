$Env:OPENOCD_SCRIPTS = "C:\Users\Wolfgang\.espressif\tools\openocd-esp32\v0.12.0-esp32-20230921\openocd-esp32\share\openocd\scripts"
$Env:PATH = "C:\Users\Wolfgang\.espressif\tools\xtensa-esp-elf-gdb\12.1_20221002\xtensa-esp-elf-gdb\bin;C:\Users\Wolfgang\.espressif\tools\riscv32-esp-elf-gdb\12.1_20221002\riscv32-esp-elf-gdb\bin;C:\Users\Wolfgang\.espressif\tools\xtensa-esp32-elf\esp-12.2.0_20230208\xtensa-esp32-elf\bin;C:\Users\Wolfgang\.espressif\tools\xtensa-esp32s2-elf\esp-12.2.0_20230208\xtensa-esp32s2-elf\bin;C:\Users\Wolfgang\.espressif\tools\xtensa-esp32s3-elf\esp-12.2.0_20230208\xtensa-esp32s3-elf\bin;C:\Users\Wolfgang\.espressif\tools\riscv32-esp-elf\esp-12.2.0_20230208\riscv32-esp-elf\bin;C:\Users\Wolfgang\.espressif\tools\esp32ulp-elf\2.35_20220830\esp32ulp-elf\bin;C:\Users\Wolfgang\.espressif\tools\cmake\3.24.0\bin;C:\Users\Wolfgang\.espressif\tools\openocd-esp32\v0.12.0-esp32-20230921\openocd-esp32\bin;C:\Users\Wolfgang\.espressif\tools\ninja\1.10.2\;C:\Users\Wolfgang\.espressif\tools\idf-exe\1.0.3\;C:\Users\Wolfgang\.espressif\tools\ccache\4.8\ccache-4.8-windows-x86_64;C:\Users\Wolfgang\.espressif\tools\dfu-util\0.11\dfu-util-0.11-win64;C:\Espressif\frameworks\esp-idf-v5.1.2\tools;C:\Users\Wolfgang\.espressif\tools\idf-git\2.43.0\cmd;C:\Users\Wolfgang\.espressif\python_env\idf5.1_py3.11_env\Scripts;C:\Users\Wolfgang\.espressif;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\Program Files\ImageMagick-7.0.11-Q16-HDRI;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files\Microsoft SQL Server\120\Tools\Binn\;C:\Program Files\Git\cmd;C:\Program Files\Common Files\Autodesk Shared\;C:\Program Files (x86)\MAVProxy;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files\NVIDIA Corporation\NVIDIA NvDLISR;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;D:\Installations\ffmpeg\bin;C:\Program Files\dotnet\;C:\Program Files\nodejs\;C:\Program Files\Docker\Docker\resources\bin;C:\Program Files\UVtools\;C:\Users\Wolfgang\AppData\Local\Microsoft\WindowsApps;C:\Users\Wolfgang\AppData\Local\gitkraken\bin;C:\Users\Wolfgang\AppData\Local\Microsoft\WindowsApps;C:\Users\Wolfgang\AppData\Local\GitHubDesktop\bin;C:\Users\Wolfgang\AppData\Roaming\npm"

Stop-Process -Name "openocd" -Force
Start-Process -FilePath "openocd" -ArgumentList "-f", "board/esp32c3-builtin.cfg" -NoNewWindow
Start-Sleep -Seconds 1
exit 0