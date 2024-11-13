#!/bin/make
export	PREFIX	:=	
export	CC	:=	$(PREFIX)gcc
export	LD	:=	$(CC)
export	OBJCOPY	:=	$(PREFIX)objcopy
export	NM	:=	$(PREFIX)nm
export	SIZE	:=	$(PREFIX)size

#-------------------------------------------------------------------------------
.SUFFIXES:
#-------------------------------------------------------------------------------
TARGET		:=	lsi-11
INCLUDES	:=	include
SOURCES		:=	src
BUILD		:=	build

OPT		:=	-O3 -g

ifdef NDEBUG
DEBUG		:=	-DNDEBUG
ASAN		:=	0
else
ASAN		:=	1
endif

ifeq ($(ASAN),1)
ASANFLG		:=	-fsanitize=address
else
ASANFLG		:=
endif


CFLAGS		+=	$(OPT) -Wall -std=c99 \
			-ffunction-sections -fdata-sections \
			$(INCLUDE) -DUNIX $(ASANFLG)

LDFLAGS		+=	-Wl,-x -Wl,--gc-sections $(OPT) $(ASANFLG)

CFILES		:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.c)))

ifneq ($(BUILD),$(notdir $(CURDIR)))
#-------------------------------------------------------------------------------
export	DEPSDIR	:=	$(CURDIR)/$(BUILD)
export	OFILES	:=	$(CFILES:.c=.o)
export	VPATH	:=	$(foreach dir,$(SOURCES),$(CURDIR)/$(dir)) $(CURDIR)
export	INCLUDE	:=	$(foreach dir,$(INCLUDES),-I$(CURDIR)/$(dir)) \
				-I$(CURDIR)/$(BUILD)
export	OUTPUT	:=	$(CURDIR)/$(TARGET)

.PHONY: $(BUILD) clean all

$(BUILD):
	@echo compiling...
	@[ -d $@ ] || mkdir -p $@
	@$(MAKE) --no-print-directory -C $(BUILD) -f $(CURDIR)/Makefile

clean:
	@echo "[CLEAN]"
	@rm -rf $(BUILD) $(TFILES) $(OFILES) $(TARGET)

$(TARGET): $(TFILES)

else

#-------------------------------------------------------------------------------
# main target
#-------------------------------------------------------------------------------
.PHONY: all

all: $(OUTPUT)

$(OUTPUT): $(TARGET).elf
	@cp $(TARGET).elf $(OUTPUT)

%.o: %.c
	@echo "[CC]    $(notdir $@)"
	@$(CC) -MMD -MP -MF $(DEPSDIR)/$*.d $(CFLAGS) -c $< -o $@
	@#$(CC) -S $(CFLAGS) -o $(@:.o=.s) $< # create assembly file

$(TARGET).elf: $(OFILES)
	@echo "[LD]    $(notdir $@)"
	@$(LD) $(LDFLAGS) $(OFILES) -o $@ -Wl,-Map=$(@:.elf=.map)

-include $(DEPSDIR)/*.d

#-------------------------------------------------------------------------------
endif
#-------------------------------------------------------------------------------
