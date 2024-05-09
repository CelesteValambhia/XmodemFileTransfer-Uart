# Compiler
CC = g++

# Compiler flags #-std=c++14
CFLAGS = -Wall -std=c++14

# Target files
TARGET = xmodem_send

# Header files
HEADERS = Log.hpp

all: $(TARGET)

$(TARGET): $(TARGET).cpp $(HEADERS)
	$(CC) $(CFLAGS) -o $(TARGET) $(TARGET).cpp

# Clean rule
clean:
	rm -f $(TARGET)