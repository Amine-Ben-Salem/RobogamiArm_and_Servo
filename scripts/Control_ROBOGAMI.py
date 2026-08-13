import csv
import datetime
import serial
import numpy as np
from serial.tools import list_ports
import time

SERIAL_COM = True  # Set to True if you want to use serial communication
baud_rate=115200


if SERIAL_COM:
    # port_list = list(list_ports.comports(include_links=False))
    
    # port_arduino = port_list[0].device
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
    port_arduino = hc06_port
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

# Setup function
#def inver():

def send_command(joint_angles_desired=None, gripper_velocity=0):
    if SERIAL_COM:
        try:
            command = str(joint_angles_desired[0]) + ' ' + str(joint_angles_desired[1]) + ' ' + str(joint_angles_desired[2]) + ' ' + str(joint_angles_desired[3]) + ' ' + str(joint_angles_desired[4]) + ' ' + str(joint_angles_desired[5]) + ' ' + str(gripper_velocity) + ' \r\n'
            ser.reset_output_buffer()
            ser.write(command.encode())
        except Exception as e:
            print("Error serial:", e)

    if SERIAL_COM:
        ser.reset_input_buffer()
        message = ""
        start_time = time.time()
        while True:
            data = ser.readline()
            data = data.decode("utf-8")

            if not data:
                if time.time() - start_time > 5:
                    raise TimeoutError("No valid response received from the device within 5 seconds.")
                continue

            message = message + data

            # uncomment this to debug the received message
            # print("Received message: ", message)

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
                # split the message by spaces to get individual elements
                elements = message.split()
                elements[0] = elements[0].replace("<", "")
                elements[-1] = elements[-1].replace(">", "")

                # get the current joint angles from the message (every second element, after removing "<" and ">")
                joint_angles_current = np.array([float(elements[0]), float(elements[2]), float(elements[4]), float(elements[6]), float(elements[8]), float(elements[10])])
                gripper_velocity_current = float(elements[12])
                gripper_load_current = float(elements[14])

                break
            

    return joint_angles_current, gripper_velocity_current, gripper_load_current


def main():
    # while (joint angles not reached):
    # go to starting position
    # 1. run inverse kinematics
    # 2. send command with the joint angles for starting position
    # 3. stop loop when starting position is reached


    # start while True: loop runs infinitely
    # 1. get the emg signal and get the joint angles 
    # 2. send command with the joint angles
    desired_angle = 70
    joint_angles_desired = np.array([np.deg2rad(desired_angle), np.deg2rad(desired_angle), np.deg2rad(desired_angle), np.deg2rad(desired_angle), np.deg2rad(desired_angle), np.deg2rad(desired_angle)])  # Example desired joint angles
    gripper_velocity_desired = 0  # positive (e.g. +20) for opening, negative (e.g. -20) for closing, 0 for no movement
    # # add update() function here

    print("Entering main loop")
    while True:
        t_start = time.time()
        joint_angles_robot, gripper_velocity_robot, gripper_load_robot = send_command(joint_angles_desired, gripper_velocity_desired)
        print("Desired Joint Angles: ", joint_angles_desired)
        print("Current Joint Angles: ", joint_angles_robot)
        print("Desired Gripper Velocities: ", gripper_velocity_desired)
        print("Current Gripper Velocity: ", gripper_velocity_robot)
        print("Current Gripper Load: ", gripper_load_robot)
        print("Loop time: ", time.time() - t_start)

if __name__ == '__main__':
    main()
    