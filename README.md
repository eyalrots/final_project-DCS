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
