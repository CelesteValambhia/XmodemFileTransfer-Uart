# File Transfer Using Xmodem Protocol via Uart Ports

* The `xmodem_send` and `xmodem_receive` directories contain the xmodem code.
* The code is written entirely in C++ using oops concepts.
* The Uart port is configured on 9600 baud rate to send the data.
* The data is sent on Uart port `/dev/pts/1` and received on `/dev/pts/2`.
  > These are virtual Uart ports since I do not have a linux device or boards which exposed physical Uart ports, I have created virual ports for testing. However, the settings will remain the same when used with physical Uart ports. \
  > Follow the instructions in `Serial_Port_Setup/Setup_Virtual_Serial_Comm.md` for setting up a virtual uart port.
* The progress is visible while sending.
* I have added my own log module for the logs. All the log messages can be supressed by setting the appropriate log level in the main function.


### Send and Receive Instructions

* The file to send is provided in the arguments.
```
./xmodem_send test.txt
```
* And the file to receive is also provided in the argument.
```
./xmodem_receive test.txt
```
> Note that the send function should be run first and then the receive function. The reason for this is, when the send function is called it first clears any garbage data on Uart port before sending. Hence, if the receive function is called before the send function, the handshake 'C' will also be cleared along with the garbage data, and one has to abort and call receive again.
>
> ToDo: Implement timeout functionality. 
