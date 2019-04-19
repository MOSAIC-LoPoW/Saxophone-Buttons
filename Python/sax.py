# following the example at https://makersportal.com/blog/2018/2/25/python-datalogger-reading-the-serial-output-from-arduino-to-analyze-data-using-pyserial
# export to CSV files can be added as is described in the example

import serial
import matplotlib
matplotlib.use("tkAgg")
import matplotlib.pyplot as plt
from bitstring import ConstBitStream
import logging

plot_window = 500
plt.ion()

button_states = [[1] * plot_window for _ in range(10)]
fig, axs = plt.subplots(10, sharex=True)
linedict = {}
for i in range(10):
    linedict["line{0}".format(i)] = axs[i].plot(button_states[i], 'r')
    axs[i].set_ylabel("B{0}".format(i))
    axs[i].tick_params(
    axis='x',          # changes apply to the x-axis
    which='both',      # both major and minor ticks are affected
    bottom=False,      # ticks along the bottom edge are off
    top=False,         # ticks along the top edge are off
    labelbottom=False) # labels along the bottom edge are off
    axs[i].set_ylim([-0.1,1.1])

fig.suptitle('Saxophone Button States')
fig.canvas.set_window_title('Button Visualizer')
fig.canvas.draw()

# configure the serial connections
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=460800,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

while True:
    try:
        ser.flushInput()
        byte = ConstBitStream(ser.read(1))
        start_byte = byte.read("uintle:8")
        #check if start byte (0xff) is found
        if(start_byte == 0xff):
            #read 10 bytes (10 button states)
            bytes = ConstBitStream(ser.read(10))
            for i in range(10):
                state = bytes.read("uintle:8")
                button_states[i].append(state)
                #only show plot window amount of data points
                button_states[i] = button_states[i][1:plot_window + 1]
                #set ydata for each sub plot
                linedict["line{0}".format(i)][0].set_ydata(button_states[i])
                #draw line
                axs[i].draw_artist(linedict["line{0}".format(i)][0])
            #flush events will draw the actual lines
            fig.canvas.flush_events()
    except Exception as e:
        logging.error(e)
        break