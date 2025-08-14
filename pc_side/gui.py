import serial
from tqdm import tqdm

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
            angle = sample[2:3]
            # insert to appropriate arrays
            distances.append(distance)
            angles.append(angle)
        else:
            print("Warning: one sample is incomplete.")
    
    return distances, angles

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
            # TODO: Analyze samples data to detect 5 objects and their length.
        
        if choice == '2':
            ser.write(choice.encode())
            angle = int(input("Insert angle: "))
            if not 0 <= angle <= 180:
                print("Angle out of range.")
                continue
            else:
                byte_data = angle.to_bytes(1, "little")
                ser.write(byte_data)
                distances = get_distance_data(ser, 20)
                print(distances)


if __name__ == '__main__':
    main()