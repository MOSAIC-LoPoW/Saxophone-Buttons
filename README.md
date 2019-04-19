# Saxophone-Buttons

This repo exists of two projects.

One project for the embedded c code for the nucleo, reading the state of the 10 buttons and sending this over a serial connection

The other project will visualize this data using matplotlib in python.

## Nucleo button state to serial
This project reads the state of 10 buttons and forwards it over a serial connection.

The serial payload is as follows:
* 0xFF (start byte)
* 0 or 1 for every button (=> 10 times)
* \n (stop byte, also acts as an enter in a serial console)

The LPUART of the nucleo is used, and is configured to use the ST-Link USB at the top of the nucleo board.

The baud rate is set at 460800 bps.

## Python button state visualizer
The python project uses matplotlib to visualize the button states.

A serial connection is set up, to read the state values:
~~~~
# configure the serial connection
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=460800,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
~~~~
This data is continuously plotted in 10 subplots (B0 -> B9).
Currently, a window of 500 data points is shown.
