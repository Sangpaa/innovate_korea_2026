# NUCLEO-F446RE: pMMG (MS5607) acquisition + EMG -> WS2812 bar + pMMG -> PWM light.
#   make            build build/emg_pmmg_led_F446.elf/.bin
#   make flash      program over the on-board ST-LINK (SWD) and reset
#   make clean
# Optional overrides, e.g.:  make WS2812_LEDS_PER_STRIP=15

TARGET  := emg_pmmg_led_F446
BUILD   := build
PREFIX  ?= arm-none-eabi-
CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size
PROGRAMMER ?= $(firstword $(wildcard /opt/st/stm32cubeide_*/plugins/*cubeprogrammer*/tools/bin/STM32_Programmer_CLI) STM32_Programmer_CLI)

HAL_DIR := Drivers/STM32F4xx_HAL_Driver/Src
HAL_SRC := hal hal_cortex hal_rcc hal_rcc_ex hal_gpio hal_exti hal_dma hal_dma_ex \
           hal_pwr hal_pwr_ex hal_flash hal_flash_ex hal_spi hal_tim hal_tim_ex \
           hal_uart hal_adc hal_adc_ex

C_SRC := $(wildcard Core/Src/*.c) $(foreach m,$(HAL_SRC),$(HAL_DIR)/stm32f4xx_$(m).c)
ASM_SRC := Core/Startup/startup_stm32f446xx.s

DEFS := -DSTM32F446xx -DUSE_HAL_DRIVER
ifdef WS2812_LEDS_PER_STRIP
DEFS += -DWS2812_LEDS_PER_STRIP=$(WS2812_LEDS_PER_STRIP)U
endif

INCS := -ICore/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc \
        -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/CMSIS/Include
MCU  := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

CFLAGS  := $(MCU) $(DEFS) $(INCS) -std=gnu11 -Os -g3 -Wall -Wextra -Wno-unused-parameter \
           -ffunction-sections -fdata-sections -MMD -MP
LDFLAGS := $(MCU) -TSTM32F446RETX_FLASH.ld --specs=nano.specs -Wl,--gc-sections \
           -Wl,-Map=$(BUILD)/$(TARGET).map -Wl,--print-memory-usage -lm

OBJS := $(addprefix $(BUILD)/,$(C_SRC:.c=.o) $(ASM_SRC:.s=.o))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(MCU) -x assembler-with-cpp -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS) STM32F446RETX_FLASH.ld
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

flash: $(BUILD)/$(TARGET).elf
	"$(PROGRAMMER)" -c port=SWD mode=UR -w $< -v -rst

clean:
	rm -rf $(BUILD)

.PHONY: all flash clean
-include $(OBJS:.o=.d)
