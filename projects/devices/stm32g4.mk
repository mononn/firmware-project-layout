CROSS_COMPILE ?= arm-none-eabi

HAL_ROOT := external/STM32CubeG4
PORT_ROOT := ports/stm32/g4

LD_SCRIPT := $(PORT_ROOT)/STM32G484VETx_FLASH.ld
LIBS += -lc -lnosys -lm

CFLAGS += \
	-mcpu=cortex-m4 \
	-mthumb \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mabi=aapcs

LDFLAGS += \
	-mcpu=cortex-m4 \
	-mthumb \
	-mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard \
	-mabi=aapcs \
	--specs=nano.specs

INCS += \
	$(PORT_ROOT) \

DEFS += \
	USE_HAL_DRIVER \
	STM32G484xx \

include $(PORT_ROOT)/hal.mk

.PHONY: flash erase gdbserver
flash: $(OUTHEX)
	pyocd flash -t $(PROJECT) $<
erase:
	pyocd erase -t $(PROJECT) --chip
gdbserver:
	$(Q)pyocd $@ -t $(PROJECT)
