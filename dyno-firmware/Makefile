CC ?= cc
ARM_CC := arm-none-eabi-gcc
HOST_CFLAGS := -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic -O2
CORE := src/core/dyno.c
LINK := src/core/link.c
ADS := src/drivers/ads131m02.c
TEST_INC := -Isrc/core -Isrc/drivers

SOURCE ?= adc
OUT := build/$(SOURCE)
TUSB := vendor/tinyusb/src
APP_SRC := src/board/main.c src/drivers/i2c_slave.c src/drivers/adc.c $(ADS) src/drivers/usb.c $(CORE) $(LINK)
USB_SRC := $(TUSB)/tusb.c $(TUSB)/common/tusb_fifo.c $(TUSB)/device/usbd.c $(TUSB)/device/usbd_control.c $(TUSB)/class/cdc/cdc_device.c $(TUSB)/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
SOURCES := $(APP_SRC) $(USB_SRC) src/board/startup.S
HEADERS := $(shell find src vendor -name '*.h')
LDSCRIPT := src/board/stm32f072_app.ld
BUILD_INPUTS := $(SOURCES) $(HEADERS) $(LDSCRIPT) Makefile tools/image.py
OBJECTS := $(addprefix $(OUT)/,$(addsuffix .o,$(SOURCES)))
INC := -Isrc/board -Isrc/core -Isrc/drivers -Ivendor/cmsis -I$(TUSB)
FLAGS := -std=gnu11 -Os -g3 -mcpu=cortex-m0 -mthumb -ffreestanding -fno-builtin -ffunction-sections -fdata-sections -Wall -Wextra -Werror -DSTM32F072xB $(INC) -include $(OUT)/build_id.h
ifeq ($(SOURCE),adc)
FLAGS += -DDYNO_USE_ADC=1
else ifneq ($(SOURCE),synthetic)
$(error SOURCE must be adc or synthetic)
endif

.PHONY: all firmware image test sanitize clean
all: firmware test

# Builds both images; `make image SOURCE=...` builds one.
firmware:
	$(MAKE) image SOURCE=adc
	$(MAKE) image SOURCE=synthetic

image: $(OUT)/dyno.bin $(OUT)/image.json

$(OUT)/build_id.h: $(BUILD_INPUTS)
	@mkdir -p $(OUT)
	python3 tools/image.py build-id --source $(SOURCE) --output $@ $(BUILD_INPUTS)

$(OUT)/%.c.o: %.c $(OUT)/build_id.h
	@mkdir -p $(dir $@)
	$(ARM_CC) $(FLAGS) -MMD -MP -c $< -o $@

$(OUT)/%.S.o: %.S $(OUT)/build_id.h
	@mkdir -p $(dir $@)
	$(ARM_CC) -mcpu=cortex-m0 -mthumb -c $< -o $@

$(OUT)/dyno.elf: $(OBJECTS) $(LDSCRIPT)
	$(ARM_CC) -mcpu=cortex-m0 -mthumb -nostdlib -Wl,--gc-sections,--build-id=none,-Map=$(OUT)/dyno.map,--print-memory-usage -T $(LDSCRIPT) $(OBJECTS) -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group -o $@

$(OUT)/dyno.bin: $(OUT)/dyno.elf
	arm-none-eabi-objcopy -O binary $< $@

$(OUT)/image.json: $(OUT)/dyno.bin $(OUT)/build_id.h
	python3 tools/image.py inspect --source $(SOURCE) --image $(OUT)/dyno.bin --output $@ --build-header $(OUT)/build_id.h

build/test/core_test: $(CORE) src/core/dyno.h tests/core_test.c Makefile
	@mkdir -p $(dir $@)
	$(CC) $(HOST_CFLAGS) $(TEST_INC) $(CORE) tests/core_test.c -o $@

build/test/protocol_test: $(CORE) $(LINK) $(ADS) src/core/dyno.h src/core/link.h src/drivers/ads131m02.h tests/frame.h tests/protocol_test.c Makefile
	@mkdir -p $(dir $@)
	$(CC) $(HOST_CFLAGS) $(TEST_INC) $(CORE) $(LINK) $(ADS) tests/protocol_test.c -o $@

build/test/stream_test: $(CORE) src/core/dyno.h tests/frame.h tests/stream_test.c Makefile
	@mkdir -p $(dir $@)
	$(CC) $(HOST_CFLAGS) $(TEST_INC) $(CORE) tests/stream_test.c -o $@

build/test/i2c_watchdog_test: tests/i2c_watchdog_test.c src/drivers/i2c_watchdog.h Makefile
	@mkdir -p $(dir $@)
	$(CC) $(HOST_CFLAGS) $(TEST_INC) tests/i2c_watchdog_test.c -o $@

test: build/test/core_test build/test/protocol_test build/test/stream_test build/test/i2c_watchdog_test
	./build/test/core_test
	./build/test/protocol_test
	./build/test/stream_test
	./build/test/i2c_watchdog_test

sanitize:
	@mkdir -p build/test
	$(CC) $(HOST_CFLAGS) -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer $(TEST_INC) $(CORE) $(LINK) $(ADS) tests/protocol_test.c -o build/test/protocol_test_sanitized
	$(CC) $(HOST_CFLAGS) -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer $(TEST_INC) $(CORE) tests/stream_test.c -o build/test/stream_test_sanitized
	ASAN_OPTIONS=detect_leaks=0 ./build/test/protocol_test_sanitized
	ASAN_OPTIONS=detect_leaks=0 ./build/test/stream_test_sanitized

clean:
	rm -rf build

-include $(OBJECTS:.o=.d)
