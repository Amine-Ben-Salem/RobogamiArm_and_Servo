import csv
import datetime
import serial
import numpy as np
from serial.tools import list_ports
import time
from Control_BASE import serial_base
from Control_ROBOGAMI import serial_robogami

SERIAL_COM = True
baud_rate = 115200

# ROBOGAMI CONTROL PARAMETERS
# desired_angle = 20  # degrees
# joint_angles_desired = np.array([np.deg2rad(desired_angle), np.deg2rad(desired_angle), np.deg2rad(desired_angle), 
#                                  np.deg2rad(desired_angle), np.deg2rad(desired_angle), np.deg2rad(desired_angle)])
# gripper_velocity_desired = 0  # positive (e.g. +20) for opening, negative (e.g. -20) for closing, 0 for no movement


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
    seen_base = False
    seen_robogami = False

    while not (seen_base and seen_robogami):
        msg = ser.readline().decode().strip()
        if not msg:
            continue
        if "ERROR" in msg:
            print(msg)
            print("The setup() function failed. Please check the Arduino code and try again.")
            exit()
        if msg == "<READY: Base initialized>":
            seen_base = True
        elif msg == "<READY: Robogami initialized>":
            seen_robogami = True
        else:
            print(msg)
    # the next print happens only if both base and robogami have been initialized successfully
    print("Devices initialized successfully!")

def update():
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    seen_base = False
    seen_robogami = False
    # Serial com
    if SERIAL_COM:
        try:
            # RESET THE STATES EACH UPDATE
            # Ask for Read current position or set goal position
            cmd = input("Do you want to control the Base or the Robogami? Type ""B"" for Base, ""R"" for Robogami: ")
            if cmd == "B":
                ser.write(b"BASE\n")
                while not seen_base:
                    msg = ser.readline().decode().strip()
                    if not msg:
                        continue
                    # print(msg)
                    if "Entering Base control mode..." in msg:
                        print(msg)
                        seen_base = True
            elif cmd == "R":
                ser.write(b"ROBOGAMI\n")
                while not seen_robogami:
                    msg = ser.readline().decode().strip()
                    if not msg:
                        continue
                    # print(msg)
                    if "Entering Robogami control mode..." in msg:
                        print(msg)
                        seen_robogami = True
            else:
                print("Invalid command. Please type 'B' for Base or 'R' for Robogami.")
                return

            if seen_base:
                serial_base(ser)
            elif seen_robogami:
                serial_robogami(ser)
            else:
                # THIS SHOULD NOT HAPPEN, BUT JUST IN CASE
                print("Neither Base nor Robogami control mode was entered. Please check the Arduino code and try again.")
                exit()
        except Exception as e:
            print("Error serial:", e)
            exit()

if __name__ == '__main__':
    while True:
        update()