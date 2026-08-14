import serial
import time
from serial.tools import list_ports

def serial_base(ser):
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # ----SEND MESSAGE-----
    try:
        # Ask for Read current position or set goal position
        cmd = input("To read position type ""R"", to set one type ""W"": ")
        if cmd == "R":
            cmd = "PresentPos"
        elif cmd == "W":
            cmd = input("Provide a goal position in deg: ")
            cmd = "GoalPos " + cmd
        # Format the message
        command = cmd + 'DONE\n'

        #print("Sending command (serial_base): ", command)

        # Send message
        ser.reset_output_buffer()
        ser.write(command.encode())

    except Exception as e:
        print("Error serial:", e)

    # ----RECEIVE MESSAGE----
    # ser.reset_input_buffer()
    message = ""
    while True:
        # Get data from TCP server
        data = ser.readline()
        data = data.decode("utf-8")

        message = message + data

        #print("Received message (serial_base): ", message)
        
        # Search for unique first element of message "<" (find returns -1 if element is not found)
        position_first_element = message.find("<")

        # Check if the unique first element is included in the data and cut everything before if so
        if position_first_element == -1:
            continue
        else:
            message = message[position_first_element:]

        # Search for the unique last element of message ">"
        position_last_element = message.find(">")

        # Check if the unique last element is included in the data and cut everything after if so
        if position_last_element == -1:
            continue
        else:
            message = message[:position_last_element+1]
            break

    print(message)