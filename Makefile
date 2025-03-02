CC = gcc
CFLAGS = -Wall -Wextra -O2 -I./include `pkg-config --cflags libusb-1.0`
LDFLAGS = -lrtlsdr -lfftw3 -lm -lusb-1.0

TARGET = rf_scanner
SRC_DIR = src
INCLUDE_DIR = include
BUILD_DIR = build

SRCS = $(SRC_DIR)/main.c
OBJS = $(BUILD_DIR)/main.o

.PHONY: all clean rf_scanner2

all: $(BUILD_DIR) $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

rf_scanner2: $(BUILD_DIR)
	$(CC) $(CFLAGS) $(SRC_DIR)/rf_scanner2.c -o rf_scanner2 $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET) rf_scanner2

lint:
	cppcheck --enable=all --suppress=missingIncludeSystem $(SRC_DIR)/*.c $(INCLUDE_DIR)/*.h

test:
	@echo "Running tests..."
	@if [ -n "$(TEST)" ]; then \
		./tests/$(TEST); \
	else \
		find ./tests -type f -executable -exec {} \;; \
	fi