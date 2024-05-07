/**
* @file		xmodem_receive.cpp
* @author	Celeste Valambhia
* @brief	Implemetation of Receiver code on Uart
*
* Uses:		/dev/pts/2 Uart serial port at 9600 baud rate
* Xmodem frames: <soh><blk#><255-blk#><128 bytes data><CRC>
* Refer to:	https://web.mit.edu/6.121/www/other/pcplot_man/pcplot14.htm
*			http://ee6115.mit.edu/amulet/xmodem.htm
*/

/***********************************************************************/
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <stdexcept>
#include <cstdint>
#include <string.h>
#include <fstream>
#include "Log.hpp" 

/* Declaring log module */
Log log_message;

/***********************************************************************/
/* Port to receive data from */
#define RECEIVE_PORT "/dev/pts/4"
//#define BAUD_RATE B9600

/* Xmodem data buffer size */
#define XDATA_BUFFER_SIZE 128
#define XPACKET_BUFFER_SIZE 133

/***********************************************************************/

class SerialPortReceiver {
private:
	const char* port_;
	const char* filename_;
	int serial_port_;
	/* Xmodem protocol flow control defines */
	static const char SOH = 0x01; // Start of Header
	static const char EOT = 0x04; // End of Transmission
	static const char ACK = 0x06; // Acknowledgement
	static const char NAK = 0x15; // Not Acknowledgement
	static const char ETB = 0x17; // End of Transmission Block (Return to Amulet OS mode)
	static const char CAN = 0x18; // Cancel (Force receiver to start sending C's)
	static const char C = 0x43;	 // ASCII “C”

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
		tty.c_cc[VMIN] = 1;  // Minimum number of characters to read
		tty.c_cc[VTIME] = 5; // Timeout in deciseconds for non-canonical read

		cfsetospeed(&tty, B9600); // Set baud rate to 9600 bps

		if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
			//throw std::runtime_error("Error configuring serial port.");
			log_message.Error("Error configuring UART serial port.");
		}
		log_message.Info("Configured Uart port ", port_, " at 9600 baud rate.");
	}

	/* CRC calculation code */
	unsigned short calcrc(char* ptr, int count) // replacing int with uint16_t
	{
		unsigned short crc;
		int i;
		crc = 0;
		while (--count >= 0) // iterate over every byte of data
		{
			crc = crc ^ (unsigned short)*ptr++ << 8; // crc is XORed with the current byte shifted left by 8 bits
			i = 8; // number of bits
			do
			{
				if (crc & 0x8000)
					crc = crc << 1 ^ 0x1021; // If the most significant bit of crc (bit 15) is set (1), it performs a bitwise XOR operation with 0x1021. Then, it shifts crc one bit to the left.
				else
					crc = crc << 1; // shifts crc one bit to the left.
			} while (--i);
		}
		log_message.Debug("CRC: ", crc);
		return crc;
	}

public:
	/* Constructor : Configures serial Uart port and initializes the Receiver */
	SerialPortReceiver(const char* port, const char* filename)
		: port_(port), filename_(filename) {
			serial_port_ = open(port_, O_RDWR);
			if (serial_port_ < 0) {
				//throw std::runtime_error("Failed to open serial port.");
				log_message.Error("Failed to open serial port.");
			}
			log_message.Info("Opened serial port for Read and Write.");
			configurePort();
		}

	/* Destructor: Closes the serial uart port */
	~SerialPortReceiver() {
		close(serial_port_);
		log_message.Info("Uart serial port closed.");
	}

	/* Copy constructor */
	SerialPortReceiver(const SerialPortReceiver& other) = delete;

	/* Copy assignment operator */
	SerialPortReceiver& operator=(const SerialPortReceiver& other) = delete;

	/* Receive data on Uart via Xmodem protocol */
	void receiveData() {
		log_message.Info("Receiving file ", filename_);

		/* Send 'C' to indicate readiness to receive data in CRC mode */
		char handshake = 'C';
		write(serial_port_, &handshake, 1);
		log_message.Info("Sent 'C' to the sender.");

		std::ofstream file(filename_, std::ios::binary); // Create the binary file
		if (!file) {
			//throw std::runtime_error("Failed to create file.");
			log_message.Error("Failed to create file");
		}

		unsigned char blk = 1; //binary number, starts at 01 increments by 1, and wraps OFFH to OOH (not to 01)
		unsigned char blk_comp = ~blk; //complement of blk i.e. 255-blk
		char buffer[XPACKET_BUFFER_SIZE]; // 128 bytes data + 3 bytes header

		log_message.Info("Receiving data...");
		while (true) {
			/* Wait for SOH */
			char soh;
			read(serial_port_, &soh, 1);
			if (soh != SOH) {
				//throw std::runtime_error("Invalid header received.");
				log_message.Error("Invalid header received.");
			}
			log_message.Debug("SOH received.");

			/* Read the packet */
			read(serial_port_, buffer, XPACKET_BUFFER_SIZE);
			log_message.Debug("Reading block: ", blk, "...");
			if (buffer[1] != blk || buffer[2] != blk_comp) {
				/* Send NAK for retransmission */
				log_message.Debug("Data corrupted, blk value does not match. Retransmission required. Sending NAK to the sender.");
				char nak = NAK;
				write(serial_port_, &nak, 1);
				continue;
			}

			/* Calculate CRC */
			log_message.Debug("Calculating CRC...");
			unsigned short crc = calcrc(buffer + 3, XDATA_BUFFER_SIZE);
			unsigned short recv_crc = (buffer[131] << 8) | buffer[132];
			if (crc != recv_crc) {
				/* Send NAK for retransmission */
				log_message.Debug("Data corrupted, CRC does not match. Retransmission required. Sending NAK to the sender.");
				char nak = NAK;
				write(serial_port_, &nak, 1);
				continue;
			}
			log_message.Debug("CRC matches, Block: ", blk, " read successfully.");

			/* Writing data to file */
			file.write(buffer + 3, XDATA_BUFFER_SIZE);
			log_message.Debug("Block: ", blk, " written to file.");

			/* Sending ACK */
			char ack = ACK;
			write(serial_port_, &ack, 1);
			log_message.Debug("Sent ACK to sender.");

			/* Move to the next block */
			++blk;
			blk_comp = ~blk;

			/* Check for EOT */
			if (buffer[0] == EOT) {
				// Send ACK for EOT
				write(serial_port_, &ack, 1);
				log_message.Debug("EOT received, sent ACK.");
				break;
			}
		}

		log_message.Info("File received successfully.");
	}

};

/***********************************************************************/
int main(int argc, char* argv[]) {
	/* Set log level */
	log_message.SetLevel(log_message.LogLevelDebug);

	if (argc != 2) {
		log_message.Error("Usage: ", argv[0], " <file_to_receive>");
		return 1;
	}

	try {
		SerialPortReceiver receiver(RECEIVE_PORT, argv[1]);
		receiver.receiveData();
	}
	catch (const std::exception& e) {
		log_message.Error("Error: ", e.what());
		return 1;
	}
	return 0;
}

