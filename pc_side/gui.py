import serial

def main():
    options = ['1']
    distance_samples = []

    menu = "choose 1 please :)"
    print(menu)

    ser = serial.Serial('COM3', 9600, bytesize=8,
                        parity=serial.PARITY_NONE, stopbits=1, timeout=5)
    
    # clear buffer
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    while True:
        choice = input("Select from menu: ")
        if choice == '1':
            ser.write(choice.encode())
            for _ in range(180):
                sample = ser.readline().decode()
                print(f"sample length: {len(sample)}, sample: {sample}")
                distance_samples.append(sample)
                continue


if __name__ == '__main__':
    main()