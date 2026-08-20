import csv
import datetime
import serial
import numpy as np
from serial.tools import list_ports
import time

def send_command(ser,joint_angles_desired=None, gripper_velocity=0):
    try:
        command = (
            str(joint_angles_desired[0]) + ' ' +
            str(joint_angles_desired[1]) + ' ' +
            str(joint_angles_desired[2]) + ' ' +
            str(joint_angles_desired[3]) + ' ' +
            str(joint_angles_desired[4]) + ' ' +
            str(joint_angles_desired[5]) + ' ' +
            str(gripper_velocity) + ' DONErobo\n'
        )
        ser.reset_output_buffer()
        ser.write(command.encode())
    except Exception as e:
        print("Error serial:", e)

    
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
        # print("Received message before parsing (control_robogami.py): ", message)
        

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

            # uncomment this to debug the received message
            # print("Received message after parsing (control_robogami.py): ", message)

            # split the message by spaces to get individual elements
            elements = message.split()
            elements[0] = elements[0].replace("<", "")
            elements[-1] = elements[-1].replace(">", "")
            # print("Elements: ", elements)

            # get the current joint angles from the message (every second element, after removing "<" and ">")
            joint_angles_current = np.array([float(elements[0]), float(elements[2]), float(elements[4]), float(elements[6]), float(elements[8]), float(elements[10])])
            gripper_velocity_current = float(elements[12])
            gripper_load_current = float(elements[14])

            break
            

    return joint_angles_current, gripper_velocity_current, gripper_load_current


def serial_robogami(ser):
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # while (joint angles not reached):
    # go to starting position
    # 1. run inverse kinematics
    # 2. send command with the joint angles for starting position
    # 3. stop loop when starting position is reached


    # start while True: loop runs infinitely
    # 1. get the emg signal and get the joint angles 
    # 2. send command with the joint angles
    # joint_angles_desired = np.array([np.deg2rad(desired_angle), np.deg2rad(desired_angle), np.deg2rad(desired_angle), 
    #                                  np.deg2rad(desired_angle), np.deg2rad(desired_angle), np.deg2rad(desired_angle)])
    
    # Ask every time the function runs
    while True:
        user_input = input('Provide a desired angle in deg (10-70), or type "RE" to reset: ').strip()

        if user_input.upper() == "RE":
            command = "RESETrobo\n"
            ser.reset_output_buffer()
            ser.write(command.encode())

            message = ""
            while True:
                data = ser.readline().decode("utf-8")
                if not data:
                    continue
                message += data
                if "Robogami initialized" in message or "<READY: Robogami initialized>" in message:
                    print("Robogami reset ongoing!")
                    loading_message()
                    return

        try:
            desired_angle_deg = float(user_input)

            if desired_angle_deg < 10:
                print("Value too low, instead setting angle to 10.")
                desired_angle_deg = 10
            elif desired_angle_deg > 70:
                print("Value too high, instead setting angle to 70.")
                desired_angle_deg = 70

            break
        except ValueError:
            print('Invalid input. Type "RE" to reset or enter a numeric angle.')
   
    joint_angles_desired = np.deg2rad(
        np.array([desired_angle_deg, desired_angle_deg, desired_angle_deg,
                desired_angle_deg, desired_angle_deg, desired_angle_deg])
    )

    gripper_velocity_desired = 0
    t_start = time.time()
    joint_angles_robot, gripper_velocity_robot, gripper_load_robot = send_command(
        ser, joint_angles_desired, gripper_velocity_desired
    )


    joint_angles_desired = np.deg2rad(
        np.array([desired_angle_deg, desired_angle_deg, desired_angle_deg,
                  desired_angle_deg, desired_angle_deg, desired_angle_deg])
    )
    
    gripper_velocity_desired = 0  # positive (e.g. +20) for opening, negative (e.g. -20) for closing, 0 for no movement

    t_start = time.time()
    joint_angles_robot, gripper_velocity_robot, gripper_load_robot = send_command(ser,joint_angles_desired, gripper_velocity_desired)
    

    print("Desired Joint Angles: ", joint_angles_desired)
    print("Current Joint Angles: ", joint_angles_robot)
    print("Desired Gripper Velocities: ", gripper_velocity_desired)
    print("Current Gripper Velocity: ", gripper_velocity_robot)
    print("Current Gripper Load: ", gripper_load_robot)

    loading_message()
    # print("Loop time: ", time.time() - t_start)

def loading_message():
    duration = 5.0          # seconds
    steps = 50              # update frequency / smoothness
    bar_width = 30          # characters

    for i in range(steps + 1):
        progress = i / steps
        percent = int(progress * 100)
        filled = int(bar_width * progress)
        bar = "#" * filled + "-" * (bar_width - filled)

        print(f"\rLoading [{bar}] {percent:3d}%", end="", flush=True)
        time.sleep(duration / steps)
    print("\nRobogami ready!\n")