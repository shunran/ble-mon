PROJECT_NAME := ble-mon

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) )

SDK_PATH = C:/nRF51_SDK_10.0.0
OPEN_OCD = C:/openocd-0.9.0/bin-x64/openocd -f interface/stlink-v2.cfg -f target/nrf51.cfg
LINKER_SCRIPT=link/ble-mon_gcc_nrf51.ld
OUTPUT_FILENAME := $(PROJECT_NAME)

TEMPLATE_PATH = $(SDK_PATH)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(SDK_PATH)/components/libraries/util/app_error.c \
$(SDK_PATH)/components/libraries/timer/app_timer.c \
$(SDK_PATH)/components/libraries/fifo/app_fifo.c \
$(SDK_PATH)/components/libraries/util/nrf_assert.c \
$(SDK_PATH)/components/libraries/uart/retarget.c \
$(SDK_PATH)/components/libraries/uart/app_uart_fifo.c \
$(SDK_PATH)/components/drivers_nrf/delay/nrf_delay.c \
$(SDK_PATH)/components/drivers_nrf/common/nrf_drv_common.c \
$(SDK_PATH)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
$(SDK_PATH)/components/drivers_nrf/uart/nrf_drv_uart.c \
$(SDK_PATH)/components/drivers_nrf/clock/nrf_drv_clock.c \
src/uart.c \
src/timer.c \
src/radio.c \
src/contact.c \
src/storage.c \
src/gpio.c \
src/main.c \
$(SDK_PATH)/components/ble/common/ble_advdata.c \
$(SDK_PATH)/components/ble/common/ble_conn_params.c \
$(SDK_PATH)/components/ble/common/ble_srv_common.c \
$(SDK_PATH)/components/toolchain/system_nrf51.c \
$(SDK_PATH)/components/softdevice/common/softdevice_handler/softdevice_handler.c \
$(SDK_PATH)/components/drivers_nrf/pstorage/pstorage.c \
$(SDK_PATH)/components/ble/ble_services/ble_nus/ble_nus.c \
$(SDK_PATH)/components/ble/ble_advertising/ble_advertising.c

#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath $(SDK_PATH)/components/toolchain/gcc/gcc_startup_nrf51.s)
#ASM_SOURCE_FILES  = $(abspath link/gcc_startup_nrf51.s)

#includes common to all targets
INC_PATHS = -I$(MAKEFILE_DIR)/src/boards
INC_PATHS += -I$(MAKEFILE_DIR)/src/config
INC_PATHS += -I$(MAKEFILE_DIR)/src
INC_PATHS += -I$(SDK_PATH)/components/libraries/util
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/pstorage
INC_PATHS += -I$(SDK_PATH)/components/toolchain/gcc
INC_PATHS += -I$(SDK_PATH)/components/toolchain
INC_PATHS += -I$(SDK_PATH)/components/ble/common
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/common
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_advertising
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/config
# INC_PATHS += -I$(abspath ../../../../../bsp)
INC_PATHS += -I$(SDK_PATH)/components/libraries/trace
INC_PATHS += -I$(SDK_PATH)/components/libraries/fifo
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/gpiote
# INC_PATHS += -I$(SDK_PATH)/components/ble/device_manager
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/uart
INC_PATHS += -I$(SDK_PATH)/components/libraries/uart
INC_PATHS += -I$(SDK_PATH)/components/device
INC_PATHS += -I$(SDK_PATH)/components/softdevice/common/softdevice_handler
INC_PATHS += -I$(SDK_PATH)/components/softdevice/s130/headers
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/delay
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/clock
INC_PATHS += -I$(SDK_PATH)/components/libraries/timer
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/hal
INC_PATHS += -I$(SDK_PATH)/components/ble/ble_services/ble_nus


OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
#CFLAGS  = -DBOARD_BLE400
CFLAGS  = -DBOARD_RADIOLAND
#CFLAGS += -DUSE_UART_LOG
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
#CFLAGS += -Wall -Werror -O3
CFLAGS += -Wall -Werror -O0 -g3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
#ASMFLAGS += -DBOARD_BLE400
#ASMFLAGS += -DSOFTDEVICE_PRESENT
#ASMFLAGS += -DNRF51
#ASMFLAGS += -DS130
#ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
#ASMFLAGS += -DSWI_DISABLE0
#ASMFLAGS += -D__HEAP_SIZE=0x399
#ASMFLAGS = -Wa,--defsym,__STACK_SIZE=0xC00

ASMFLAGS +=-D__STACK_SIZE=0xB00
# 0xB00
# 0xC92
ASMFLAGS += -D__HEAP_SIZE=0x500
ASMFLAGS += -DNRF51
# original 0x800
#default target - first one defined
default: clean ble-mon

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e ble-mon

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	ble-mon
	@echo 	flash_softdevice


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

ble-mon: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<


#$(NO_ECHO)$(AS) $(ASMFLAGS) -c -o $@ $<
# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<



# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
	$(OPEN_OCD) -c "program $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex verify reset exit"

## Flash softdevice
flash_softdevice:
	@echo Flashing: s130_nrf51_8.0.0_softdevice.hex
	$(OPEN_OCD) -c "program $(SDK_PATH)/components/softdevice/s130/hex/s130_nrf51_1.0.0_softdevice.hex verify reset exit"

## Flash softdevice
reset:
	@echo Resetting device
	$(OPEN_OCD) -c "init;reset run;exit"


## Start OpenOCD daemon for manual commands via telnet
openocd: 
	$(OPEN_OCD)