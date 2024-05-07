/***********************************************************************/
/**
 * @file	xmodem_send.cpp
 * @author	Celeste Valambhia
 * @brief	Implemetation of Xmodem Sender code on Uart
 *
 * Uses:	/dev/pts/1 Uart serial port at 9600 baud rate
 * Xmodem frames: <soh><blk#><255-blk#><128 bytes data><CRC>
 * Refer to:https://web.mit.edu/6.121/www/other/pcplot_man/pcplot14.htm
 *			http://ee6115.mit.edu/amulet/xmodem.htm
 *
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
/* Port to send data from */
#define SEND_PORT "/dev/pts/3"
//#define BAUD_RATE B9600

/* Xmodem data buffer size */
#define XDATA_BUFFER_SIZE 128
#define XPACKET_BUFFER_SIZE 133

/***********************************************************************/

class SerialPortSender {
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
	static const char C = 0x43;	 // ASCII C

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
		tty.c_lflag = 0;		// Turn off canonical mode
		tty.c_oflag = 0;		// Turn off output processing
		tty.c_cc[VMIN] = 1;		// Minimum number of characters to read
		tty.c_cc[VTIME] = 5;	// Timeout in deciseconds for non-canonical read

		cfsetospeed(&tty, B9600); // Set baud rate

		if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
			//throw std::runtime_error("Error configuring UART serial port.");
			log_message.Error("Error configuring UART serial port.");
		}
		log_message.Info("Configured Uart port ", port_, "at 9600 baud rate.");
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
	/* Constructor : Configures serial Uart port and initializes the Sender */
	SerialPortSender(const char* port, const char* filename)
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
	~SerialPortSender() {
		close(serial_port_);
		log_message.Info("Uart serial port closed.");
	}

	/* Copy constructor */
	SerialPortSender(const SerialPortSender& other) = delete;

	/* Copy assignment operator */
	SerialPortSender& operator=(const SerialPortSender& other) = delete;


	/* Send data on Uart via Xmodem protocol */
	void sendData() {
		log_message.Info("Sending file ", filename_);

		std::ifstream file(filename_, std::ios::binary); //opening the file in binary mode, which is suilable for reading binary data such as images, executables, or any files
		if (!file) {
			//throw std::runtime_error("Failed to open file.");
			log_message.Error("Failed to open file. Cross-check file name and permissions");
		}

		/* Get the file size */
		file.seekg(0, std::ios::end);
		long long file_size = file.tellg();
		file.seekg(0, std::ios::beg);
		log_message.Info("File size is: ", file_size);

		/* Wait for 'C' from receiver */
		char handshake;
		read(serial_port_, &handshake, 1);
		if (handshake != C) {
			//throw std::runtime_error("Failed to receive 'C' handshake from receiver.");
			log_message.Error("Failed to receive 'C' handshake from receiver.");
		}
		log_message.Info("Received 'C' handshake from receiver.");

		unsigned char blk = 1; //binary number, starts at 01 increments by 1, and wraps OFFH to OOH (not to 01)
		unsigned char blk_comp = ~blk; //complement of blk i.e. 255-blk
		char buffer[XPACKET_BUFFER_SIZE]; // 128 bytes data + 3 bytes header

		/* <soh><blk#><255-blk#><128 bytes data><2 bytes CRC> */
		/* Total bytes 1 + 1 + 1 + 128 + 2  = 133 */
		long long bytes_sent = 0;
		while (!file.eof()) {
			buffer[0] = SOH;
			//std::cout << buffer[0] << std::endl;
			buffer[1] = blk;
			//std::cout << buffer[1] << std::endl;
			buffer[2] = blk_comp;
			//std::cout << buffer[2] << std::endl;
			std::cout << "SOH: " << static_cast<int>(SOH) << ", blk: " << static_cast<int>(blk) << ", blk_comp: " << static_cast<int>(blk_comp) << std::endl;

			file.read(buffer + 3, XDATA_BUFFER_SIZE);
			int bytes_read = file.gcount();
			log_message.Debug("Number of bytes read in the file: ", bytes_read);

			if (bytes_read < XDATA_BUFFER_SIZE) {
				// Pad the remaining bytes with 0x1A (Ctrl+Z)
				memset(buffer + 3 + bytes_read, 0x1A, XDATA_BUFFER_SIZE - bytes_read);
				log_message.Debug("Padding the remaining bytes with 0x1A.");
			}

			/* Calculate CRC of data */
			unsigned short crc = calcrc(buffer + 3, XDATA_BUFFER_SIZE);
			buffer[131] = crc >> 8; // The 16-bit CRC value is divided into two bytes. The MSB is stored in buffer[131]. The `>>` operator shifts the CRC value 8 bits to the right, to extract the MSB.
			buffer[132] = crc & 0xFF; // The LSB is stored here. The `& 0xFF` operation ensures that only the 8 least significant bits of the CRC value are retained, effectively masking out any higher-order bits that may have been shifted into the LSB position during the previous step.

			log_message.Debug("The Xmodem packet frame is: \n");
			log_message.Debug("<SOH: ", buffer[0], "><blk: ", buffer[1], "><255-blk: ", buffer[2], "><128 byte data><CRC: ", buffer[131], " ", buffer[132], ">");

			log_message.Info("Writing the frames on serial port.");
			write(serial_port_, buffer, 133);
			bytes_sent += XDATA_BUFFER_SIZE;

			/* Calculate progress in percentage */
			int progress = static_cast<int>((bytes_sent * 100) / file_size);
			log_message.Info("Progress: ", progress, " %");

			/* Waiting for response from receiver */
			char response;
			read(serial_port_, &response, 1);
			if (response == NAK) {
				log_message.Debug("Response NAK received from the receiver. Resending the data block.");
				continue; // Resend the block
			}
			else if (response == ACK) {
				log_message.Debug("Response ACK received from the receiver. Sending the next data block.");
				++blk; // Move to the next block
				blk_comp = ~blk;
			}
			else {
				//throw std::runtime_error("Unexpected response from receiver.");
				log_message.Error("Unexpected response from receiver.");
			}

		}

		/* Send EOT */
		char eot = EOT;
		log_message.Debug("Sending EOT.");
		write(serial_port_, &eot, 1);

		/* Wait for ACK of EOT*/
		char response;
		read(serial_port_, &response, 1);
		if (response != ACK) {
			//throw std::runtime_error("Failed to receive ACK after sending EOT.");
			log_message.Error("Failed to receive ACK after sending EOT.");
		}
		log_message.Debug("Received ACK for EOT.");
		log_message.Info("File Sent SUCCESSFULLY!");
	}

};

/***********************************************************************/
int main(int argc, char* argv[]) {
	/* Set log level */
	log_message.SetLevel(log_message.LogLevelDebug);

	if (argc != 2) {
		log_message.Error("Usage: ", argv[0], " <file_to_send>");
		return 1;
	}

	try {
		SerialPortSender sender(SEND_PORT, argv[1]);
		sender.sendData();
	}
	catch (const std::exception& e) {
		log_message.Error("Error: ", e.what());
		return 1;
	}
	return 0;
}
