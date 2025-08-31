# Project Structure

This project is divided into three main directories: `header`, `pc_side`, and `source`.

## `header` directory

This directory contains the header files for the MSP430 microcontroller.

*   `api.h`: Header file for the application programming interface.
*   `bsp.h`: Header file for the board support package.
*   `halGPIO.h`: Header file for the hardware abstraction layer for GPIO.
*   `msp_header/`: This directory contains MSP430-specific header files.
    *   `in430.h`: MSP430-specific intrinsic functions.
    *   `intrinsics.h`: MSP430-specific intrinsic functions.
    *   `iomacros.h`: Macros for I/O register access.
    *   `msp430g2553.h`: Header file for the MSP430G2553 microcontroller.

## `pc_side` directory

This directory contains the PC-side application and related files.

*   `gui.py`: Python script for the graphical user interface.
*   `radar_plot.png`: Image of a radar plot.
*   `script.txt`: A script file.
*   `Library/`: This directory contains various text files.

## `source` directory

This directory contains the source code for the MSP430 microcontroller.

*   `api.c`: Source file for the application programming interface.
*   `bsp.c`: Source file for the board support package.
*   `halGPIO.c`: Source file for the hardware abstraction layer for GPIO.
*   `main.c`: Main source file for the application.

## Function Descriptions

This section describes the functions in `source/api.c`.

*   `init_sample_array`: Initializes an integer array with zeros.
*   `get_distance_from_sensor`: Measures distance using the ultrasonic sensor.
*   `get_distance_from_ldrs`: Measures light intensity from two LDRs to estimate distance.
*   `move_motor_to_new_angle`: Moves the servo motor to a specified angle.
*   `wait_for_motor`: Waits for the motor to settle after a movement.
*   `calculate_distance_ldr`: Calculates distance based on the average of two LDR readings.
*   `go_to_zero`: Moves the motor to the 0-degree (home) position.
*   `scan_with_motor`: Scans a range of angles, taking distance measurements at each step.
*   `scan_with_sonic`: Performs a 180-degree scan using the ultrasonic sensor.
*   `scan_with_ldr`: Performs a 180-degree scan using the LDRs.
*   `scan_at_given_angle`: Continuously measures distance at a user-specified angle.
*   `erase_info`: Erases file information from flash memory segments.
*   `inc_lcd`: Displays an incrementing number on the LCD for testing purposes.
*   `dec_lcd`: Displays a decrementing number on the LCD for testing purposes.
*   `rra_lcd`: Rotates a character across the LCD display for testing purposes.
*   `script_mode`: Enters script execution mode, running commands from a stored file.
*   `find_next_text_header`: Finds the header of the next text file stored in flash memory.
*   `read_file`: Reads a file from flash memory and displays its content on the LCD.
*   `file_mode`: Enters file browsing mode to select and read text files.
*   `calibrate_ldr`: Calibrates the LDR sensors and stores the calibration data.
*   `read_calibration`: Reads and displays the stored LDR calibration data.
*   `detect_both_sensors`: Scans 180 degrees, measuring distance with both ultrasonic and LDR sensors.
