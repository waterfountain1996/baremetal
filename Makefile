PREFIX      := arm-none-eabi-
CC          := $(PREFIX)gcc
OBJCOPY     := $(PREFIX)objcopy

CFLAGS      := -Wall -Wextra -Wundef -std=c99
LDFLAGS     := -Tlinkerscript.ld -nostartfiles -nolibc -nostdlib
TARGETFLAGS := -ffreestanding -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

SRC         := blinky.c vector.c
OBJ         := $(SRC:%.c=obj/%.o)
FIRMWARE    := blinky

all: $(FIRMWARE).bin $(FIRMWARE).elf

$(FIRMWARE).bin: $(FIRMWARE).elf
	$(OBJCOPY) -O binary $< $@

$(FIRMWARE).elf: $(OBJ)
	$(CC) $(LDFLAGS) $(TARGETFLAGS) -o $@ $(OBJ)

obj/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(TARGETFLAGS) -c $< -o $@

clean:
	rm -rf obj/ $(FIRMWARE).elf $(FIRMWARE).bin
