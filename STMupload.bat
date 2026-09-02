@echo off
setlocal

rem .\STMupload.batで実行

:loop

set "PROJECT_ROOT=%~dp0"
cd /d "%PROJECT_ROOT%"

rem ボード名を取得
for /f "tokens=*" %%a in ('dir /b /ad ".pio\build"') do (
    set "BOARD=%%a"
    goto :found
)

:found
if "%BOARD%"=="" (
    echo [ERROR] ビルド済みのボードが見つかりません。
    pause
    exit /b
)

echo Target Board: %BOARD%

rem PlatformIOでビルドを実行
call C:\Users\PC_User\.platformio\penv\Scripts\platformio.exe run

rem 書き込みの実行
echo Writing firmware...
cd /d "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin"

STM32_Programmer_CLI.exe -c port=SWD -w "%PROJECT_ROOT%.pio\build\%BOARD%\firmware.elf" -rst

rem Enterキーを押してループ
set a=""
set /p a=
goto loop
