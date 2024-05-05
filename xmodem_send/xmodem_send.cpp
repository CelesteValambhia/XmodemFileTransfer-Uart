/**
* @file		xmodem_send.cpp
* @author	Celeste Valambhia
* @brief	Implemetation of Xmodem Sender code on Uart
*
* Uses:		/dev/pts/1 Uart serial port at 9600 baud rate
* Xmodem frames: <soh><blk#><255-blk#><128 bytes data><CRC>
* Refer to: https://web.mit.edu/6.121/www/other/pcplot_man/pcplot14.htm
*			http://ee6115.mit.edu/amulet/xmodem.htm
*
*/

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <stdexcept>

class SerialPortSender {
private:
    const char* port_;
	const char* filename_;
    int serial_port_;

	/* Xmodem protocol flow control defines */
	static const char SOH = 0x01;


	/* Configure Uart port */
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

public:
    SerialPortSender(const char* port, const char* filename)
		: port_(port), filename_(filename) {
        serial_port_ = open(port_, O_RDWR);
        if (serial_port_ < 0) {
            throw std::runtime_error("Failed to open serial port.");
        }
        configurePort();
    }

    // Destructor
    ~SerialPortSender() {
        close(serial_port_);
    }

    // Copy constructor
    SerialPortSender(const SerialPortSender& other) = delete;

    // Copy assignment operator
    SerialPortSender& operator=(const SerialPortSender& other) = delete;

	// Send Data
    void sendData(const char* data) const {
        ssize_t bytes_written = write(serial_port_, data, strlen(data));
        if (bytes_written < 0) {
            throw std::runtime_error("Error writing to serial port.");
        }
    }

};

int main() {
    try {
        SerialPort port("/dev/pts/1");
        const char* data = "Hello, UART!";
        port.sendData(data);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}

