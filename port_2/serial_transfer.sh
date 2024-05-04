#!/bin/bash

# Check command-line arguments
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <send/receive> <file> <serial_port>"
    exit 1
fi

MODE="$1"
FILE="$2"
PORT="$3"

# Function to send file
send_file() {
    FILE="$1"
    PORT="$2"

    # Send the file over the serial port
    cat "$FILE" > "$PORT"
}

# Function to receive file
receive_file() {
    FILE="$1"
    PORT="$2"

    # Receive the file from the serial port
    cat "$PORT" > "$FILE"
}

case "$MODE" in
    send)
        send_file "$FILE" "$PORT"
        ;;
    receive)
        receive_file "$FILE" "$PORT"
        ;;
    *)
        echo "Invalid mode. Use 'send' or 'receive'."
        exit 1
        ;;
esac

echo "File transfer complete."

