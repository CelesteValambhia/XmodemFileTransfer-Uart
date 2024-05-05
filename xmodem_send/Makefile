# Compiler
CC = g++

# Compiler flags #-std=c++14
CFLAGS = -Wall -std=c++14

# Target files
TARGET = xmodem_send

all: $(TARGET)

$(TARGET): $(TARGET).cpp
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET).cpp

# Clean rule
clean:
	rm -f $(TARGET)
