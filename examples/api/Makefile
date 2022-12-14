#
# (C) Copyright 2007 Semihalf
#
# SPDX-License-Identifier:	GPL-2.0+
#

ifeq ($(ARCH),powerpc)
LOAD_ADDR = 0x40000
endif
ifeq ($(ARCH),arm)
ifneq ($(filter $(SOC),tz3000 tz2000),)
LOAD_ADDR = 0x80200000
else
LOAD_ADDR = 0x1000000
endif
endif

include $(TOPDIR)/config.mk

# Resulting ELF and binary exectuables will be named demo and demo.bin
OUTPUT-$(CONFIG_API) = $(obj)demo
OUTPUT = $(OUTPUT-y)

# Source files located in the examples/api directory
SOBJ_FILES-$(CONFIG_API) += crt0.o
COBJ_FILES-$(CONFIG_API) += demo.o
COBJ_FILES-$(CONFIG_API) += glue.o
COBJ_FILES-$(CONFIG_API) += libgenwrap.o

# Source files which exist outside the examples/api directory
EXT_COBJ_FILES-$(CONFIG_API) += lib/crc32.o
EXT_COBJ_FILES-$(CONFIG_API) += lib/ctype.o
EXT_COBJ_FILES-$(CONFIG_API) += lib/div64.o
EXT_COBJ_FILES-$(CONFIG_API) += lib/string.o
EXT_COBJ_FILES-$(CONFIG_API) += lib/time.o
EXT_COBJ_FILES-$(CONFIG_API) += lib/vsprintf.o
ifeq ($(ARCH),powerpc)
EXT_SOBJ_FILES-$(CONFIG_API) += arch/powerpc/lib/ppcstring.o
endif

# Create a list of source files so their dependencies can be auto-generated
SRCS	+= $(addprefix $(SRCTREE)/,$(EXT_COBJ_FILES-y:.o=.c))
SRCS	+= $(addprefix $(SRCTREE)/,$(EXT_SOBJ_FILES-y:.o=.S))
SRCS	+= $(addprefix $(SRCTREE)/examples/api/,$(COBJ_FILES-y:.o=.c))
SRCS	+= $(addprefix $(SRCTREE)/examples/api/,$(SOBJ_FILES-y:.o=.S))

# Create a list of object files to be compiled
OBJS	+= $(addprefix $(obj),$(SOBJ_FILES-y))
OBJS	+= $(addprefix $(obj),$(COBJ_FILES-y))
OBJS	+= $(addprefix $(obj),$(notdir $(EXT_COBJ_FILES-y)))
OBJS	+= $(addprefix $(obj),$(notdir $(EXT_SOBJ_FILES-y)))

CPPFLAGS += -I..

all:	$(obj).depend $(OUTPUT)

#########################################################################

$(OUTPUT):	$(OBJS)
		$(LD) -Ttext $(LOAD_ADDR) -o $@ $^ $(PLATFORM_LIBS)
		$(OBJCOPY) -O binary $@ $(OUTPUT).bin 2>/dev/null

# Rule to build generic library C files
$(obj)%.o: $(SRCTREE)/lib/%.c
	$(CC) -g $(CFLAGS) -c -o $@ $<

# Rule to build architecture-specific library assembly files
$(obj)%.o: $(SRCTREE)/arch/$(ARCH)/lib/%.S
	$(CC) -g $(CFLAGS) -c -o $@ $<

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
