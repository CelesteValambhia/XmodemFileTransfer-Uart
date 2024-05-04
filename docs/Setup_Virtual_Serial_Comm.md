# Setup Virtual Serial Uart Port on Linux

* First install socat utility
  ```
  sudo apt-get install socat
  ```

* Create two virtual serial ports with debug mode on
  ```
  socat -d -d pty,rawer,echo=0 pty,rawer,echo=0
  ```
