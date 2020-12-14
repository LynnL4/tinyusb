CFLAGS += \
  -flto \
  -mthumb \
  -mabi=aapcs \
  -mcpu=cortex-m7 \
  -mfloat-abi=hard \
  -mfpu=fpv5-d16 \
  -nostdlib -nostartfiles \
  -DSTM32H725xx \
  -DCFG_TUSB_MCU=OPT_MCU_STM32H7 \
  -DBOARD_DEVICE_RHPORT_NUM=1


CFLAGS += -DBOARD_DEVICE_RHPORT_SPEED=OPT_MODE_FULL_SPEED


# suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=maybe-uninitialized -Wno-error=cast-align

ST_FAMILY = h7
ST_CMSIS = hw/mcu/st/cmsis_device_$(ST_FAMILY)
ST_HAL_DRIVER = hw/mcu/st/stm32$(ST_FAMILY)xx_hal_driver

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/$(BOARD)/STM32H725AEIX_FLASH.ld

SRC_C += \
	$(ST_CMSIS)/Source/Templates/system_stm32$(ST_FAMILY)xx.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_cortex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_rcc.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_rcc_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_gpio.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_uart.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_pwr_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_hal_pcd.c \
	$(ST_HAL_DRIVER)/Src/stm32$(ST_FAMILY)xx_ll_usb.c \

SRC_S += \
	$(ST_CMSIS)/Source/Templates/gcc/startup_stm32h725xx.s

INC += \
	$(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
	$(TOP)/$(ST_CMSIS)/Include \
	$(TOP)/$(ST_HAL_DRIVER)/Inc \
	$(TOP)/hw/bsp/$(BOARD)

# For TinyUSB port source
VENDOR = st
CHIP_FAMILY = synopsys

# For freeRTOS port source
FREERTOS_PORT = ARM_CM7/r0p1

# For flash-jlink target
JLINK_DEVICE = stm32h725ae

# flash target using on-board stlink
flash: flash-stlink
