@echo off
cd /d %~dp0

echo replace-freertos-file...
C:
cd %userprofile%\STM32Cube\Repository\STM32Cube_FW_F4_V*\Middlewares\Third_Party\FreeRTOS\Source\portable\GCC\ARM_CM4F
@echo on
copy /y .\port.c %~dp0\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM4F
copy /y .\portmacro.h %~dp0\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM4F
@echo off

cd /d %~dp0
echo -------------------Build Start----------------------


