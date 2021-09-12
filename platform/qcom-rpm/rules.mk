LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

ARCH := arm
ARM_CPU := cortex-m3

MEMBASE := 0x90000
ROMBASE := 0x00000000
MEMSIZE := 0x10000

MODULE_SRCS += \
	$(LOCAL_DIR)/init.c \
	$(LOCAL_DIR)/debug.c \
	$(LOCAL_DIR)/vectab.c \
	$(LOCAL_DIR)/qtimer.c \
	$(LOCAL_DIR)/sysaccess.c \
	$(LOCAL_DIR)/uart_dm.c

LINKER_SCRIPT += \
	$(BUILDDIR)/system-twosegment.ld

include make/module.mk

