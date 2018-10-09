# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

# compile ASM with arm-none-eabi-gcc
# compile C with /usr/bin/clang
ASM_FLAGS =  -x assembler-with-cpp  -mthumb -mcpu=cortex-m3  -O3  -g  

ASM_DEFINES = -DIS_STM32=1 -DSTM32F103xB -DUSE_HAL_DRIVER -D__packed="__attribute__((__packed__))" -D__weak="__attribute__((weak))"

ASM_INCLUDES = -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Application/Inc -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Drivers/CMSIS/Device/ST/STM32F1xx/Include -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Drivers/CMSIS/Include -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Drivers/STM32F1xx_HAL_Driver/Inc -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Inc -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Middlewares/Third_Party/FreeRTOS/Source/include -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 

C_FLAGS = -target arm-none-eabi -Wno-unknown-attributes  -I/usr/lib/gcc/arm-none-eabi/8.2.0/include -I/usr/lib/gcc/arm-none-eabi/8.2.0/include-fixed -I/usr/lib/gcc/arm-none-eabi/8.2.0/../../../../arm-none-eabi/include  -target arm-none-eabi -std=gnu99 -dM -fno-common -fmessage-length=0 -Wall -fno-exceptions -ffunction-sections -fdata-sections -fomit-frame-pointer -fshort-enums  -I/usr/lib/gcc/arm-none-eabi/8.2.0/include -I/usr/lib/gcc/arm-none-eabi/8.2.0/include-fixed -I/usr/lib/gcc/arm-none-eabi/8.2.0/../../../../arm-none-eabi/include -Wno-unknown-attributes  -mthumb -mcpu=cortex-m3  -O3  -g -DDEBUG -g -gdwarf-2  

C_DEFINES = -DIS_STM32=1 -DSTM32F103xB -DUSE_HAL_DRIVER -D__packed="__attribute__((__packed__))" -D__weak="__attribute__((weak))"

C_INCLUDES = -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Application/Inc -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Drivers/CMSIS/Device/ST/STM32F1xx/Include -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Drivers/CMSIS/Include -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Drivers/STM32F1xx_HAL_Driver/Inc -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Inc -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Middlewares/Third_Party/FreeRTOS/Source/include -I/home/hector/Documentos/Robotica/firmware/minisumo-stm32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 
