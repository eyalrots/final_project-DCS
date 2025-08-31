import os
import time
import serial
import math
import matplotlib.pyplot as plt
from tqdm import tqdm

#-------------------------------servo------------------------------------------
# def recive_object_loction(distances, angles, threshold=150):
#     objects = []
#     object_distance = []
#     object_angle = []
#     prev_distance = distances[0]
#     prev_angle = angles[0]
#     big_jump = 20
    
#     for i in range(1, (len(distances)-1)):
#         if prev_distance < threshold:
#             object_distance.append(prev_distance)
#             object_angle.append(prev_angle)
#         if  abs(prev_angle-distances[i]) >= big_jump and len(object_distance) > 0:
#             angle_span = (object_angle[-1] - object_angle[0]) % 180
#             center_angle = (object_angle[0] + angle_span/2.0) % 180
#             center_distance = object_distance[len(object_distance)//2]
#             length_cm = center_distance * math.pi * angle_span / 180.0
#             if length_cm == 0: length_cm = 1  # avoid zero-length
#             objects.append((length_cm, center_angle, center_distance))
#             object_distance = []
#             object_angle = []
#         prev_distance = distances[i]
#         prev_angle = distances[i]
    
#     # flush last cluster if still open
#     if len(object_distance) > 0:
#         if prev_distance < threshold:
#             object_distance.append(prev_distance)
#             object_angle.append(prev_angle)
#         angle_span = (object_angle[-1] - object_angle[0]) % 180
#         center_angle = (object_angle[0] + angle_span/2.0) % 180
#         center_distance = object_distance[len(object_distance)//2]
#         length_cm = center_distance * math.pi * angle_span / 180.0
#         objects.append((length_cm, center_angle, center_distance))
    
#     return objects
import math

def _calculate_object_properties(readings):
    """Helper function to process a list of (angle, distance) readings."""
    # Extract angles and distances
    object_angles = [r[0] for r in readings]
    object_distances = [r[1] for r in readings]

    # Calculate the angle span and center angle
    angle_span = abs(object_angles[-1] - object_angles[0])
    center_angle = object_angles[0] + angle_span / 2.0
    
    # Use the median distance for robustness against outliers
    sorted_distances = sorted(object_distances)
    center_distance = sorted_distances[len(sorted_distances) // 2]
    
    # Calculate object length using the arc length formula
    # length = radius * angle_in_radians
    angle_in_radians = math.radians(angle_span)
    length_cm = center_distance * angle_in_radians
    
    return (length_cm, center_angle, center_distance)

def detect_objects(distances, angles, distance_threshold=50.0, jump_threshold=10.0):
    """
    Detects objects from a sensor scan based on distance and angle readings.

    Args:
        distances (list): List of distance measurements from the sensor.
        angles (list): List of corresponding angle measurements.
        distance_threshold (float): Readings above this distance are ignored.
        jump_threshold (float): A jump in distance greater than this value
                                signifies a gap between objects.

    Returns:
        list: A list of tuples, where each tuple represents an object
              in the format (length_cm, center_angle, center_distance).
    """
    objects = []
    current_object_readings = []
    
    # Combine angles and distances into pairs for easier iteration
    readings = zip(angles, distances)

    for angle, distance in readings:
        # Check if the point is part of a potential object
        if distance < distance_threshold:
            # If this is the start of a new object, just add the point
            if not current_object_readings:
                current_object_readings.append((angle, distance))
            else:
                # Check if there's a large jump from the previous point
                prev_distance = current_object_readings[-1][1]
                if abs(distance - prev_distance) < jump_threshold:
                    # If the jump is small, it's the same object
                    current_object_readings.append((angle, distance))
                else:
                    # A large jump means the previous object has ended
                    if len(current_object_readings) > 1: # Require at least 2 points
                        obj = _calculate_object_properties(current_object_readings)
                        objects.append(obj)
                    # Start a new object with the current point
                    current_object_readings = [(angle, distance)]
        else:
            # If the distance is too far, the object (if any) has ended
            if len(current_object_readings) > 1:
                obj = _calculate_object_properties(current_object_readings)
                objects.append(obj)
            # Reset for the next object
            current_object_readings = []

    # After the loop, process any remaining object
    if len(current_object_readings) > 1:
        obj = _calculate_object_properties(current_object_readings)
        objects.append(obj)
        
    return objects


# --- plotting function (one dot per object, labeled with summary) ---
def plot_objects(distances, angles):
    objs = detect_objects(distances, angles)

    fig = plt.figure(figsize=(8, 4.5))
    ax = plt.subplot(111, projection="polar")

    # configure radar style
    ax.set_theta_zero_location("E")   # 0° on the right
    ax.set_theta_direction(1)         # counter-clockwise
    ax.set_thetalim(0, math.pi)       # 0–180°
    ax.set_rlim(0, 400)

    # angle labels
    ax.set_xticks([0, math.pi/6, math.pi/3, math.pi/2,
                   2*math.pi/3, 5*math.pi/6, math.pi])
    ax.set_xticklabels(["0°","30°","60°","90°","120°","150°","180°"])
    ax.set_yticks([50, 100, 200, 400])
    ax.set_yticklabels(["50cm","100cm","200cm","400cm"])

    # plot one dot per object
    for length_cm, angle_deg, distance_cm in objs:
        angle_rad = math.radians(angle_deg)
        ax.plot([angle_rad], [distance_cm], "ro", markersize=8)
        ax.text(angle_rad, distance_cm,
                f"({distance_cm:.1f}cm,{angle_deg:.1f}°,{length_cm:.1f}cm)",
                ha="left", va="bottom",fontsize=6)

    plt.title("Detected Objects (0-180°)/n (distance_cm, angle_deg, length_cm)")
    plt.tight_layout()
    plt.show()

def telemeter(ser,command='2'):
    while(True):
        angle = int(input("Insert angle:/n (enter -1 to exit) "))
        if angle == -1:
            break

        if 0 <= angle <= 180:
            ser.write(command.encode())
            byte_data = angle.to_bytes(1, "little")
            ser.write(byte_data)
            distances = get_distance_data(ser, 20)
            print(distances)
        
        else:
            print("Angle out of range.")
#-----

def get_distance_data(ser, length):
    raw_samples = []
    distances = []
    angles = []

    print("In distance function...")
    # receive all sampled data from MCU
    received = 0
    size = 0
    with tqdm(total=length, unit='samples', desc='Processing Samples') as pbar:
        while received <= length:
            if (ser.in_waiting > 0):
                size = ser.in_waiting
                data_received = ser.read(ser.in_waiting)
                raw_samples.append(data_received)
                received += size
                pbar.update(size)
    
        # clean buffer if not empty
        if (ser.in_waiting > 0):
            size = ser.in_waiting
            data_received = ser.read(ser.in_waiting)
            raw_samples.append(data_received)
            received += size
            pbar.update(size)

    print(received)
    # prepare data
    # flatten samples into a one-dimentional array
    raw_samples = [item for sublist in raw_samples for item in sublist]
    # make two arrays -> 1. distances ; 2. angles
    for i in range (0, len(raw_samples), 3):
        if i+3 <= len(raw_samples):
            sample = raw_samples[i:i+3]
            # split 2-bytes for distance and 1-byte for angle
            distance = int.from_bytes(sample[0:2], byteorder="little")
            angle = sample[2]
            # insert to appropriate arrays
            distances.append(distance)
            angles.append(angle)
        else:
            print("Warning: one sample is incomplete.")
    
    return distances, angles


def send_file(ser: serial.Serial, file_name: str):
    """
    Sends a file with a custom 10-byte header over an existing serial connection.

    The function constructs and sends a header containing:
    1. size (2 bytes): File size in bytes, little-endian. Max 65535 bytes.
    2. name (7 bytes): File name, truncated or padded with nulls.
    3. type (1 byte): A constant value of 0.
    
    Args:
        ser (serial.Serial): An already-opened pyserial object.
        file_name (str): The path to the file to be sent.

    Returns:
        bool: True if sending was successful, False otherwise.
    """
    # --- 1. Validate file and get its properties ---
    try:
        file_size = os.path.getsize(file_name)
    except FileNotFoundError:
        print(f"Error: File '{file_name}' not found.")
        return False

    if file_size > 2048:
        print(f"Error: File size ({file_size} bytes) exceeds the 2K byte limit.")
        return False
        
    print(f"File: '{os.path.basename(file_name)}', Size: {file_size} bytes")
    time.sleep(2)
    # --- 2. Construct the 10-byte header ---
    # Part 1: File Size (2 bytes, little-endian)
    header = file_size.to_bytes(2, byteorder='little')
    
    # Part 2: File Name (7 bytes, truncated or padded)
    name_bytes = os.path.basename(file_name).encode('utf-8')
    header += name_bytes[:7].ljust(7, b'\x00')
    
    # Part 3: File Type (1 byte, value 0)
    header += (0).to_bytes(1, byteorder='little')
    
    # --- 3. Send data ---
    try:
        # Send the header
        print(f"Sending header over '{ser.name}'...")
        # ser.write(header)
         # --- THIS IS THE MODIFIED SECTION ---
        print(f"Sending header over '{ser.name}' one byte at a time...")
        for byte_of_header in header:
            ser.write(bytes([byte_of_header])) # Convert int back to a bytes object
            print(f"  > Sent header byte: {hex(byte_of_header)}")
            time.sleep(0.01) # Wait for 1 second
        print(header.decode())
        time.sleep(0.01)
        # Send the file content
        print("Sending file content...")
        bytes_sent = 0
        with open(file_name, 'rb') as f:
            # while chunk := f.read(64): # Read in chunks for better performance
            #     ser.write(chunk)
            #     bytes_sent += len(chunk)
            while True:
                byte_to_send = f.read(1) # Read a single byte
                if not byte_to_send:
                    break # End of file
                ser.write(byte_to_send)
                print(f"  > Sent file byte: {hex(byte_to_send[0])}")
                time.sleep(0.05)
                bytes_sent += 1
        
        # ser.flush()
        print(f"Successfully sent {bytes_sent} bytes.")
        return True

    except Exception as e:
        print(f"An error occurred during transmission: {e}")
        return False

def main():
    options = ['1']
    state_2_flag = 1

    menu = "choose 1 please :)"
    print(menu)

    ser = serial.Serial('COM3', 9600, bytesize=8,
                        parity=serial.PARITY_NONE, stopbits=1, timeout=5)
    
    # clear buffer
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    while True:
        # clear buffer
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        choice = input("Select from menu: ")
        if choice == '1':
            ser.write(choice.encode())
            distances, angles = get_distance_data(ser, 540)
            print(f"distances: {distances[0:5]} angles: {angles[0:5]}")
            plot_objects(distances, angles)
        
        if choice == '2':
            # ser.write(choice.encode())
            # angle = int(input("Insert angle: "))
            # if not 0 <= angle <= 180:
            #     print("Angle out of range.")
            #     continue
            # else:
            #     byte_data = angle.to_bytes(1, "little")
            #     ser.write(byte_data)
            #     distances = get_distance_data(ser, 20)
            #     print(distances)
            telemeter(ser)

        if choice == '3':
                ser.write(choice.encode())
                distances, angles = get_distance_data(ser, 540)
                print(f"distances: {distances[0:5]} angles: {angles[0:5]}")
                plot_objects(distances, angles)

        if choice == '4':
                ser.write(choice.encode())
                send_file(ser, "test1.txt")
                time.sleep(1)
                ack = ser.read(1)
                print(ack)
                # ser.write(choice.encode())
                # send_file(ser, "test2.txt")
                # time.sleep(1)
                # ser.write(choice.encode())
                # send_file(ser, "test3.txt")
        
        if choice == '5':
            print("state 5")
            ser.write(choice.encode())

        if choice == '6':
            print("state 6")
            ser.write(choice.encode())

        if choice == '7':
            print("state 7")
            ser.write(choice.encode())


if __name__ == '__main__':
    main()