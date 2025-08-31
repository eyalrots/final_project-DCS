import os
import sys
import threading
import time
import serial
import math
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm
from scipy.signal import find_peaks
from matplotlib.gridspec import GridSpec
import struct

#-------------------------------constants---------------------------------------
FILES = ["test1.txt", "test2.txt", "test3.txt", "text2.txt", "text3.txt", "text4.txt", 
         "scr1.txt", "scr2.txt", "scr3.txt", "scr4.txt"]
SCRIPTS = ["scr1.txt", "scr2.txt", "scr3.txt", "scr4.txt"]
OPCODES = {
    'inc_lcd':   1, 'dec_lcd':   2, 'rra_lcd':   3, 'set_delay': 4,
    'clear_lcd': 5, 'servo_deg': 6, 'servo_scan':7, 'sleep':     8,
}
OPERANDS = {
    'inc_lcd':1, 'dec_lcd':1, 'rra_lcd':1, 'set_delay':1,
    'clear_lcd':0, 'servo_deg':1, 'servo_scan':2, 'sleep':0,
}
LDR_CALIB = [200, 240, 260, 270, 285, 320, 330, 350, 380, 400]

#-------------------------------servo------------------------------------------
def _calculate_object_properties(readings):
    """Helper function to process a list of (angle, distance) readings."""
    object_angles = [r[0] for r in readings]
    object_distances = [r[1] for r in readings]

    start_angle = object_angles[0]
    end_angle = object_angles[-1]
    angle_span = abs(end_angle - start_angle)
    center_angle = start_angle + angle_span / 2.0
    
    center_distance = np.median(object_distances)
    
    angle_in_radians = math.radians(angle_span)
    length_cm = center_distance * angle_in_radians
    
    return (length_cm, center_angle, center_distance, start_angle, end_angle)

def detect_objects(distances, angles, distance_threshold=80.0, jump_threshold=25.0, min_points_for_object=3):
    """
    Detects objects using a more robust median-based comparison.
    """
    objects = []
    current_object_readings = []
    
    readings = zip(angles, distances)

    for angle, distance in readings:
        if distance < distance_threshold:
            if not current_object_readings:
                current_object_readings.append((angle, distance))
            else:
                # *** KEY LOGIC CHANGE ***
                # Compare to the MEDIAN distance of the object, not just the last point.
                # This is much more robust to noise and outliers.
                median_distance = np.median([r[1] for r in current_object_readings])
                
                if abs(distance - median_distance) < jump_threshold:
                    # Point is close to the object's center, add it
                    current_object_readings.append((angle, distance))
                else:
                    # A large jump signifies the end of the previous object
                    if len(current_object_readings) >= min_points_for_object:
                        obj = _calculate_object_properties(current_object_readings)
                        objects.append(obj)
                    # Start a new object with the current point
                    current_object_readings = [(angle, distance)]
        else:
            # Distance is too far, end the current object
            if len(current_object_readings) >= min_points_for_object:
                obj = _calculate_object_properties(current_object_readings)
                objects.append(obj)
            current_object_readings = []

    # After the loop, process any final remaining object
    if len(current_object_readings) >= min_points_for_object:
        obj = _calculate_object_properties(current_object_readings)
        objects.append(obj)
        
    return objects

def plot_objects(distances, angles):
    """
    Detects and plots objects using a stable two-panel layout.
    """
    objs = detect_objects(distances, angles)

    # --- Corrected Plotting Setup ---
    # Use GridSpec to create a figure with two different subplot types
    fig = plt.figure(figsize=(14, 7))
    gs = GridSpec(1, 2, width_ratios=[2.5, 1])
    fig.suptitle("Object Detection Scan", fontsize=16)

    # Create the polar subplot for the radar chart
    ax = fig.add_subplot(gs[0], projection='polar')

    # Create the standard subplot for the text panel
    text_ax = fig.add_subplot(gs[1])
    # --- End of Correction ---

    # --- Configure the Radar Plot (left panel) ---
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_thetalim(0, math.pi)
    ax.set_rlabel_position(22.5)
    ax.grid(True)
    ax.spines['polar'].set_visible(False)

    # --- Plot Raw Sensor Data ---
    angles_rad = np.deg2rad(angles)
    valid_indices = [i for i, d in enumerate(distances) if d < 150]
    ax.scatter(np.array(angles_rad)[valid_indices], np.array(distances)[valid_indices], 
               c=np.array(distances)[valid_indices], cmap='winter', 
               s=10, alpha=0.7)

    # --- Plot Detected Objects and Prepare Labels ---
    object_info_text = "Detected Objects:\n---------------------\n"
    if not objs:
        object_info_text += "None"
    else:
        for i, (length_cm, center_angle, distance_cm, start_angle, end_angle) in enumerate(objs):
            arc_angles_rad = np.deg2rad(np.linspace(start_angle, end_angle, 50))
            ax.plot(arc_angles_rad, [distance_cm] * 50, color='red', linewidth=5, alpha=0.8)

            label_angle_rad = np.deg2rad(center_angle)
            ax.text(label_angle_rad, distance_cm + 15, f"#{i+1}", 
                    ha="center", va="center", fontweight='bold',
                    bbox=dict(boxstyle="circle,pad=0.3", fc="yellow", ec="black", lw=1))

            object_info_text += (f"#{i+1}:\n"
                                 f"  Length: {length_cm:.1f} cm\n"
                                 f"  Distance: {distance_cm:.1f} cm\n"
                                 f"  Angle: {center_angle:.1f}°\n\n")

    # --- Configure the Text Panel (right panel) ---
    text_ax.axis('off')
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    text_ax.text(0.05, 0.95, object_info_text, transform=text_ax.transAxes, 
                 fontsize=12, verticalalignment='top', bbox=props)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

def telemeter(ser, command='2'):
    if (command=='2'):
        angle = int(input("Insert angle:/n (enter -1 to exit) "))
        ser.write(command.encode())
        ser.write(angle.to_bytes(1, "little"))
    distance = 1
    while (distance!=0):
        distance = ser.read(2)
        if not distance:
            break
        print(f"{int.from_bytes(distance, byteorder="little"):03}\r", end="")


def read_from_serial_port(ser, length=900):
    """
    Reads a specific number of bytes from a serial port into one large bytearray.

    This function continuously checks the serial port's input buffer and reads
    data in chunks as it becomes available, extending a single bytearray
    until the desired number of bytes (`length`) has been received.

    Args:
        ser: An active pySerial serial port object.
        length: The total number of bytes to read.

    Returns:
        A bytearray containing the concatenated data read from the port.
    """
    # Use bytearray() to efficiently build one single, large array of bytes.
    raw_samples = bytearray()
    received = 0
    
    print(f"Attempting to read {length} bytes from the serial port...")

    # Using tqdm for a progress bar is helpful for user feedback.
    # Note: You must have tqdm installed (`pip install tqdm`)
    from tqdm import tqdm
    with tqdm(total=length, unit='B', unit_scale=True, desc='Receiving Data') as pbar:
        while received < length:
            # Check how many bytes are waiting in the input buffer
            if ser.in_waiting > 0:
                # Read all available bytes from the buffer
                data_received = ser.read(ser.in_waiting)
                
                # Use .extend() to add the new bytes to the end of our bytearray.
                # This is the correct way to build a single array.
                # Using .append() would create a list of byte objects.
                raw_samples.extend(data_received)
                
                received += len(data_received)
                pbar.update(len(data_received))
            else:
                # A small sleep prevents this loop from using 100% CPU
                # while waiting for data.
                time.sleep(0.01)

    print("\nFinished reading data.")
    # If more data was received than expected, trim it.
    return raw_samples[:length]

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

def parse_sensor_data(data: bytes) -> tuple[list[int], list[int], list[int]]:
    """
    Parses a 900-byte array from a sensor into three lists.

    The input data is expected to contain 180 samples, with each sample
    being 5 bytes long.

    The structure for each 5-byte sample is as follows:
    - Byte 1      (index 0):   Angle (unsigned 8-bit integer)
    - Bytes 2, 3  (index 1-2): Sonic Distance (unsigned 16-bit integer, little-endian)
    - Bytes 4, 5  (index 3-4): LDR Data (unsigned 16-bit integer, little-endian)

    Args:
        data: A bytes or bytearray object of exactly 900 bytes.

    Returns:
        A tuple containing three lists: (angles, distances, ldr_values).
        - angles: A list of 180 angle values.
        - distances: A list of 180 sonic distance values.
        - ldr_values: A list of 180 LDR sensor values.
        
    Raises:
        ValueError: If the input data is not 900 bytes long.
    """
    if len(data) != 900:
        raise ValueError(f"Input data must be 900 bytes long, but got {len(data)} bytes.")

    angles = []
    distances = []
    ldr_values = []

    # Define the format string for a single 5-byte sample.
    # '<' specifies little-endian byte order.
    # 'B' is for an unsigned char (1 byte) -> angle.
    # 'H' is for an unsigned short (2 bytes) -> distance and ldr_value.
    sample_format = '<BHH'
    sample_size = struct.calcsize(sample_format)  # This will be 5 bytes

    # Iterate through the byte array in chunks of 5 bytes
    for i in range(0, len(data), sample_size):
        # Get a 5-byte chunk for one complete sample
        chunk = data[i:i + sample_size]
        
        # Unpack the chunk into the three variables according to the format
        angle, distance, ldr_value = struct.unpack(sample_format, chunk)
        
        angles.append(angle)
        distances.append(distance)
        ldr_values.append(ldr_value)

    return angles, distances, ldr_values



def assemble_script(input_filename, output_filename="script.txt"):
    """
    Assembles a text script into a binary file of one-byte commands and operands.

    Args:
        input_filename (str): The name of the source text file.
        output_filename (str): The name of the binary file to create.
        
    Returns:
        bool: True on success, False on failure.
    """
    print(f"Assembling '{input_filename}' -> '{output_filename}'...")
    try:
        with open(input_filename, 'r') as infile, open(output_filename, 'wb') as outfile:
            for line_num, line in enumerate(infile, 1):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue

                # --- THIS IS THE FIX ---
                # Replace commas with spaces to handle both as delimiters
                line = line.replace(',', ' ')

                parts = line.split()
                command = parts[0]

                if command not in OPCODES:
                    print(f"Error on line {line_num}: Unknown command '{command}'")
                    return False
                
                opcode = OPCODES[command]
                num_expected_operands = OPERANDS[command]

                if len(parts) - 1 != num_expected_operands:
                    print(f"Error on line {line_num}: Command '{command}' expects "
                          f"{num_expected_operands} operands, but got {len(parts) - 1}.")
                    return False

                outfile.write(opcode.to_bytes(1, 'little'))

                for i in range(num_expected_operands):
                    operand_str = parts[i + 1]
                    try:
                        operand_int = int(operand_str)
                        if not 0 <= operand_int <= 255:
                            raise ValueError("Operand out of range for a single byte.")
                        outfile.write(operand_int.to_bytes(1, 'little'))
                    except ValueError:
                        print(f"Error on line {line_num}: Operand '{operand_str}' "
                              f"is not a valid integer between 0 and 255.")
                        return False
                        
    except FileNotFoundError:
        print(f"Error: Input file '{input_filename}' not found.")
        return False
        
    print("Assembly successful.")
    return True


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
    # --- 0. Text or Script? ---
    if (os.path.basename(file_name) in SCRIPTS):
        print(f"script file: {os.path.basename(file_name)}")
        assemble_script(file_name)
        file_name = "script.txt"
        f_type = 1
    else:
        f_type = 0
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
    time.sleep(0.01)
    # --- 2. Construct the 10-byte header ---
    # Part 1: File Size (2 bytes, little-endian)
    header = file_size.to_bytes(2, byteorder='little')
    
    # Part 2: File Name (7 bytes, truncated or padded)
    name_bytes = os.path.basename(file_name).encode('utf-8')
    header += name_bytes[:7].ljust(7, b'\x00')
    
    # Part 3: File Type (1 byte)
    header += (f_type).to_bytes(1, byteorder='little')
    
    # --- 3. Send data ---
    try:
        # Send the header
        print(f"Sending header over '{ser.name}'...")
        for byte_of_header in header:
            ser.write(bytes([byte_of_header])) # Convert int back to a bytes object
            time.sleep(0.01) # Wait for 1 second
        time.sleep(0.01)
        # Send the file content
        print("Sending file content...")
        bytes_sent = 0
        with open(file_name, 'rb') as f:
            while True:
                byte_to_send = f.read(1) # Read a single byte
                if not byte_to_send:
                    break # End of file
                ser.write(byte_to_send)
                time.sleep(0.05)
                bytes_sent += 1
        
        # ser.flush()
        print(f"Successfully sent {bytes_sent} bytes.")
        return True

    except Exception as e:
        print(f"An error occurred during transmission: {e}")
        return False


def _get_distance_from_reading(reading, calibration_data):
    """
    Converts a raw sensor reading to distance (cm) using linear interpolation.

    Args:
        reading (float): The sensor value to convert.
        calibration_data (list): The list of calibration values.

    Returns:
        float: The estimated distance in centimeters.
    """
    # Create the corresponding distance points (5cm, 10cm, ..., 50cm)
    calib_distances_cm = np.arange(5, 51, 5)

    # --- Handle edge cases (extrapolation) ---
    # If the reading is weaker than the farthest point, it's > 50cm
    if reading <= calibration_data[0]:
        return 50.0  # Or float('inf') if you prefer
    # If the reading is stronger than the closest point, it's < 5cm
    if reading >= calibration_data[-1]:
        return 5.0

    # --- Find the two points to interpolate between ---
    for i in range(len(calibration_data) - 1):
        lower_calib_val = calibration_data[i]
        upper_calib_val = calibration_data[i+1]

        if lower_calib_val <= reading < upper_calib_val:
            # We found the segment where our reading falls.
            lower_dist = calib_distances_cm[i]
            upper_dist = calib_distances_cm[i+1]

            # --- Perform linear interpolation ---
            # Calculate how far our reading is into the calibration segment (as a percentage)
            fraction = (reading - lower_calib_val) / (upper_calib_val - lower_calib_val)
            
            # Apply that same fraction to the distance segment
            interpolated_distance = lower_dist + fraction * (upper_dist - lower_dist)
            
            return interpolated_distance
            
    # This should not be reached if edge cases are handled, but as a fallback:
    return 50.0

def find_light_sources(measurements, calibration_data):
    """
    Finds the locations of up to two light sources from sensor readings.

    Args:
        measurements (list or np.array): An array of 181 sensor readings, one per degree.
        calibration_data (list): The list of calibration values.

    Returns:
        list: A list of tuples, where each tuple is (distance_cm, angle_deg).
    """
    measurements = np.array(measurements)
    light_sources = []
    min_significant_reading = calibration_data[0]

    # --- NEW: Apply a smoothing filter to the data to reduce noise ---
    # A moving average filter with a window of 5 samples.
    window_size = 5
    smoothed_measurements = np.convolve(measurements, np.ones(window_size)/window_size, mode='valid')
    # The output is shorter, so we need to offset indices later.
    index_offset = (window_size - 1) // 2

    # 1. Find all significant peaks in the SMOOTHED data.
    #    - Increased prominence to be more selective after smoothing.
    peak_indices_smoothed, properties = find_peaks(
        smoothed_measurements,
        height=min_significant_reading,
        distance=20,
        prominence=30
    )

    # 2. If no valid peaks are found, return an empty list
    if len(peak_indices_smoothed) == 0:
        return []

    # 3. Adjust peak indices back to the original array coordinates
    peak_indices_original = peak_indices_smoothed + index_offset

    # 4. Get the sensor readings from the ORIGINAL unsmoothed data at the peak locations
    peak_heights = measurements[peak_indices_original]

    # 5. Combine peaks and their heights, then sort by height to find the strongest ones
    found_peaks = sorted(zip(peak_heights, peak_indices_original), key=lambda x: x[0], reverse=True)

    # 6. Process up to the two strongest peaks
    for peak_reading, peak_angle in found_peaks[:2]:
        distance_cm = _get_distance_from_reading(peak_reading, calibration_data)
        light_sources.append((distance_cm, peak_angle))

    # 7. Sort the final results by angle for consistent output
    light_sources.sort(key=lambda x: x[1])
    
    return light_sources

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
            telemeter(ser)

        if choice == '3':
                ser.write(choice.encode())
                distances, angles = get_distance_data(ser, 540)
                print(f"distances: {distances[0:5]} angles: {angles[0:5]}")
                sources = find_light_sources(distances, LDR_CALIB)
                print(sources)

        if choice == '4':
                file_name = str(input("select file name to send: "))
                while (file_name not in FILES):
                    print(f"{file_name} does not exists")
                    file_name = str(input("select file name to send: "))
                ser.write(choice.encode())
                file_name = os.path.join(".", "Library", file_name)
                print(f"sending file {file_name}")
                send_file(ser, file_name)
                ack = ser.read(1)
                if (ack==b'0x15'):
                    print("Not enough space!")
                    break
                else:
                    print("file saved successfully!")
                if (os.path.basename(file_name) in SCRIPTS):
                    cmd = ser.read(1)
                    while (cmd != b'\x03'):
                        print(cmd)
                        # --- servo scan ---
                        if (cmd == b'\x01'):
                            opr_1 = ser.read(1)
                            opr_2 = ser.read(1)
                            print(f"opr 1: {opr_1} :: opr 2: {opr_2}")
                            num_of_samples = int.from_bytes(opr_2) - int.from_bytes(opr_1)
                            distances, angles = get_distance_data(ser, num_of_samples*3)
                            print(f"distances: {distances[0:5]} angles: {angles[0:5]}")
                            plot_objects(distances, angles)
                        # --- telemeter ---
                        elif (cmd == b'\x02'):
                            telemeter(ser, 0)
                        # --- read next command ---
                        cmd = ser.read(1)
        
        if choice == '5':
            print("state 5")
            ser.write(choice.encode())

        if choice == '6':
            print("state 6")
            ser.write(choice.encode())

        if choice == '7':
            ser.write(choice.encode())
            byte_data = read_from_serial_port(ser, 900)
            angles, distances, ldr_data = parse_sensor_data(byte_data)
            print(f"distances: {distances[0:5]} angles: {angles[0:5]}")
            plot_objects(distances, angles)
            sources = find_light_sources(ldr_data, LDR_CALIB)
            print(sources)


        if choice == '8':
            print("LDR2 calibration...")
            ser.write(choice.encode())


if __name__ == '__main__':
    main()