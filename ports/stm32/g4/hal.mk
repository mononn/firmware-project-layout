ST_SRCS += \
	$(PORT_ROOT)/startup_stm32g484xx.s \
	$(PORT_ROOT)/system_stm32g4xx.c \
	$(PORT_ROOT)/stm32g4xx_hal_msp.c \
	$(PORT_ROOT)/stm32g4xx_it.c \
	\
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_qspi.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c \

ST_INCS += \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Inc \
	$(HAL_ROOT)/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy \
	$(HAL_ROOT)/Drivers/CMSIS/Include \
	$(HAL_ROOT)/Drivers/CMSIS/Device/ST/STM32G4xx/Include \

$(addprefix $(OUTDIR)/, $(ST_SRCS:%=%.o)): CFLAGS+=-Wno-error

SRCS += $(ST_SRCS)
INCS += $(ST_INCS)
