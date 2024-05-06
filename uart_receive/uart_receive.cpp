/**
* @file uart_receive.cpp
* @author Celeste Valambhia
* @brief Implemetation of Sender code on Uart
*
* Uses: /dev/pts/1 Uart serial port at 9600 baud rate
*
*/

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <stdexcept>

class SerialPortReceiver {
public:
	// Constructor
    SerialPortReceiver(const char* port) : port_(port) {
        serial_port_ = open(port_, O_RDWR);
        if (serial_port_ < 0) {
            throw std::runtime_error("Failed to open serial port.");
        }
        configurePort();
    }

    // Destructor
    ~SerialPortReceiver() {
        close(serial_port_);
    }

    // Copy constructor
    SerialPortReceiver(const SerialPortReceiver& other) = delete;

    // Copy assignment operator
    SerialPortReceiver& operator=(const SerialPortReceiver& other) = delete;

    std::string receiveData() const {
        char buffer[256];
        ssize_t bytes_read = read(serial_port_, buffer, sizeof(buffer));
        if (bytes_read < 0) {
            throw std::runtime_error("Error reading from serial port.");
        }
        return std::string(buffer, bytes_read);
    }

private:
    const char* port_;
    int serial_port_;

	// Configure uart port
    void configurePort() {
        struct termios tty;
        tcgetattr(serial_port_, &tty);
        tty.c_cflag &= ~PARENB;  // Disable parity
        tty.c_cflag &= ~CSTOPB;  // Use one stop bit
        tty.c_cflag &= ~CSIZE;   // Clear the mask
        tty.c_cflag |= CS8;      // Set data bits to 8
        tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Turn on receiver, ignore modem control lines
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
        tty.c_lflag = 0; // Turn off canonical mode
        tty.c_oflag = 0; // Turn off output processing
        tty.c_cc[VMIN] = 0;  // Minimum number of characters to read
        tty.c_cc[VTIME] = 5; // Timeout in deciseconds for non-canonical read

        cfsetospeed(&tty, B9600); // Set baud rate to 9600 bps

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            throw std::runtime_error("Error configuring serial port.");
        }
    }
};

int main() {
    try {
        SerialPortReceiver receiver("/dev/pts/2");
        std::string data = receiver.receiveData();
        std::cout << "Received data: " << data << std::endl;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}

