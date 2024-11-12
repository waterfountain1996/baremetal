PREFIX   := arm-none-eabi-
CC       := $(PREFIX)gcc

CFLAGS   := -Wall -Wextra -std=c99
LDFLAGS  :=

SRC      := blinky.c
OBJ      := $(SRC:%.c=obj/%.o)

build: $(OBJ)

$(OBJ): $(SRC)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf obj/
