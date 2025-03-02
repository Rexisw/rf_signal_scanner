CC = gcc
CFLAGS = -Wall -Wextra -I../include
LDFLAGS = -lrtlsdr -lfftw3 -lm

SRC_DIR = ..
TEST_SOURCES = $(wildcard *.c)
TEST_TARGETS = $(TEST_SOURCES:.c=)

# List of source files excluding main.c
SRC_FILES = $(filter-out $(SRC_DIR)/src/main.c, $(wildcard $(SRC_DIR)/src/*.c))

.PHONY: all clean

all: $(TEST_TARGETS)

%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(SRC_FILES) $(LDFLAGS)

clean:
	rm -f $(TEST_TARGETS)