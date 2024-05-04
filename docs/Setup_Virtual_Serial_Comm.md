# Setup Virtual Serial Uart Port on Linux

* First install socat utility:
  ```
  sudo apt-get install socat
  ```

* Create two virtual serial ports with debug mode on:
  ```
  socat -d -d pty,rawer,echo=0 pty,rawer,echo=0
  ```
  ![Create Ports](images/create_two_ports.jpg)


	> [!NOTE]
    > The `socat` command is a networking tool that establishes bidirectional data streams between two endpoints. In this case, the command creates a pair of virtual pseudo-terminal (pty) devices and connects them together.

	> Here's a breakdown of the command:
    > * `socat`: This is the command-line utility for establishing data streams between two endpoints.
    > * `-d -d`: These are options to enable debugging output. It means "debug twice", providing more verbose output for troubleshooting purposes.
    > * `pty,rawer,echo=0`: This specifies the parameters for the first pseudo-terminal device (PTY). 
    > * `pty`: Indicates that a pseudo-terminal device should be created.
    > * `rawer`: Sets the mode of the pseudo-terminal to raw, meaning that input and output are processed byte-by-byte without any special interpretation.
    > * `echo=0`: Disables echo on the pseudo-terminal, meaning that characters typed by the user will not be echoed back.

* For the initial proof of concept transfer files using the script `serial_transfer.sh` via the serial ports.

* To send file via port `/dev/pts/1` use the following command:
  ```
  ./serial_transfer.sh send images.jpeg /dev/pts/1
  ```

* To receive the file on another serial port `/dev/pts/2` use the following command:
  ```
  ./serial_transfer.sh receive images.jpeg /dev/pts/2
  ```

* FANTASTIC! We have two virtusl ports for serial UART communication for file transfer. Now we can utilize these ports to transfer files via xmodem protocol.