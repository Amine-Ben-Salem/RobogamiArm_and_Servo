import serial
import time
from serial.tools import list_ports

SERIAL_COM = True
baud_rate = 115200

# -------------BLUETOOTH CONNECTION----------
if SERIAL_COM:

    # ----this is a hack to connect quickly on windows knowing the mac adress. Uncomment if using windows!
    HC06_MAC = "98:da:60:00:cf:9e"
    ports = serial.tools.list_ports.comports()  # scan all com ports
    hc06_port = None
    # Look for the HC-06 by matching the MAC address
    for port in ports:
        if port.hwid and HC06_MAC.replace(":", "").upper() in port.hwid.upper():
            hc06_port = port.device
            break
    if hc06_port is None:
        raise Exception(
            "HC-06 not found. Make sure it is paired and turned on.")
    print(f"Using HC-06 on port: {hc06_port}")

    #Linux
    # hc06_port = "/dev/rfcomm0"

    # ----Open the Bluetooth serial port
    port_arduino = hc06_port  # "use "COM" on Windows or "/dev/rfcomm*" on linux
    ser = serial.Serial(port_arduino, baud_rate, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print("Bluetooth serial port opened successfully")
    ser.write(b"START\n")
    arduino_ready = False
    while not arduino_ready:
        msg = ser.readline().decode().strip()
        if not msg:
            continue
        if "ERROR" in msg:
            print(msg)
            print("The setup() function failed. Please check the Arduino code and try again.")
            exit()
        elif msg == "<READY>":
            arduino_ready = True
            print("Setup successfully completed !")
        else:
            print(msg)

def update():
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    # ----SEND MESSAGE-----
    if SERIAL_COM:
        try:
            # Ask for Read current position or set goal position
            cmd = input("To read position type ""R"", to set one type ""W"": ")
            if cmd == "R":
                cmd = "PresentPos"
            elif cmd == "W":
                cmd = input("Provide a goal position in deg: ")
                cmd = "GoalPos " + cmd
            # Format the message
            command = cmd + '\n'

            # Send message
            ser.reset_output_buffer()
            ser.write(command.encode())

        except Exception as e:
            print("Error serial:", e)

    # ----RECEIVE MESSAGE----
    if SERIAL_COM:
        #ser.reset_input_buffer()
        message = ""
        while True:
            # Get data from TCP server
            data = ser.readline()
            data = data.decode("utf-8")

            message = message + data

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


if __name__ == '__main__':
    while True:
        update()
