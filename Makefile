CC = gcc
CFLAGS = -Wall -Wextra -O2 -I./include `pkg-config --cflags libusb-1.0`
LDFLAGS = -lrtlsdr -lfftw3 -lm -lusb-1.0

TARGET = rf_scanner
SRC_DIR = src
INCLUDE_DIR = include
BUILD_DIR = build

SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

.PHONY: all clean

all: $(BUILD_DIR) $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

lint:
	cppcheck --enable=all --suppress=missingIncludeSystem $(SRC_DIR)/*.c $(INCLUDE_DIR)/*.h

test:
	@echo "Running tests..."
	@if [ -n "$(TEST)" ]; then \
		./tests/$(TEST); \
	else \
		find ./tests -type f -executable -exec {} \;; \
	fi