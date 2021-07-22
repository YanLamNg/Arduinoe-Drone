from serial import Serial
import time
import datetime as dt
import json
import matplotlib.pyplot as plt

import matplotlib.animation as animation
import numpy as np
import threading

X_PLOT_SIZE = 40

c = threading.Condition()
flag = 0
val = 20

ys = []
lines = []
xs = range(X_PLOT_SIZE)


SERIAL_FREQUENCE = 57600
PORT = '/dev/ttyACM0'


class plotLine():
    def __init__(self, ax, label, serial_string):
        self.ax = ax
        self.label = label
        self.serial_string = serial_string
        self.line, = self.ax.plot([], [], lw=1, label=label)
        self.ys = [0]*X_PLOT_SIZE


class serialThread(threading.Thread):
    SERIAL_FREQUENCE = 57600
    PORT = '/dev/ttyACM0'

    def __init__(self, name):
        threading.Thread.__init__(self)
        print('init serial')
        self.name = name
        self.arduinoSerial = Serial(PORT, SERIAL_FREQUENCE, timeout=5);

        with self.arduinoSerial:  # the reset part is actually optional but the sleep is nice to have either way.
            self.arduinoSerial.setDTR(False)
            time.sleep(0.5)
            self.arduinoSerial.flushInput()
            self.arduinoSerial.setDTR(True)

        # reopen the serial, but this time with proper baudrate. This is the correct and working connection.
        self.arduinoSerial = Serial(PORT, SERIAL_FREQUENCE, timeout=5)


    def run(self):
        global flag
        global lines


        while True:
            json_string = self.arduinoSerial.readline().decode('utf-8')

            json_data = json.loads(json_string)
            for line in lines:
                line.ys.append(json_data[line.serial_string])
                line.ys = line.ys[-40:]



def animate(i):
    global xs, lines
    for line in lines:
        temp_ys = np.array(line.ys)
        line.line.set_data(xs, temp_ys)




if __name__ == "__main__":

    fig, ax = plt.subplots(2, 2)
    ax[0][0].set_xlim([0,41])
    ax[0][0].set_ylim([-60, 60])

    lines.append(plotLine(ax[0][0], "line 1", 'x_angle'))
    lines.append(plotLine(ax[0][0], "line 2", 'y_angle'))
    lines.append(plotLine(ax[0][0], "line 2", 'z_angle'))

    ax[0][1].set_xlim([0, 41])
    ax[0][1].set_ylim([-4, 4])
    lines.append(plotLine(ax[0][1], "line 3", 'acc_x'))
    lines.append(plotLine(ax[0][1], "line 4", 'acc_y'))
    lines.append(plotLine(ax[0][1], "line 5", 'acc_z'))

    ax[1][0].set_xlim([0, 41])
    ax[1][0].set_ylim([-3, 3])
    lines.append(plotLine(ax[1][0], "line 6", 'gyro_x'))
    lines.append(plotLine(ax[1][0], "line 7", 'gyro_y'))
    lines.append(plotLine(ax[1][0], "line 8", 'gyro_z'))

    ax[1][0].set_xlim([0, 41])
    ax[1][0].set_ylim([-3, 3])
    lines.append(plotLine(ax[1][0], "line 9", 'gyro_x'))
    lines.append(plotLine(ax[1][0], "line 10", 'gyro_y'))
    lines.append(plotLine(ax[1][0], "line 11", 'gyro_z'))

    ser_thread = serialThread('arduino')
    ser_thread.start()
    # ax[0][1].set_xlim([0, 41])
    # ax[0][1].set_ylim([-4, 4])
    # line2, = ax[0][1].plot([], [], lw=1, label="line 3")
    # line3, = ax[0][1].plot([], [], lw=1, label="line 4")
    # line4, = ax[0][1].plot([], [], lw=1, label="line 5")
    #
    # ax[1][0].set_xlim([0, 41])
    # ax[1][0].set_ylim([-3, 3])
    # line5, = ax[1][0].plot([], [], lw=1, label="line 6")
    # line6, = ax[1][0].plot([], [], lw=1, label="line 7")
    # line7, = ax[1][0].plot([], [], lw=1, label="line 8")
    #
    # lines.

    anim = animation.FuncAnimation(fig, animate, frames=200, interval=20)


    plt.show()
    ser_thread.join()

    #
    # # ax1 = fig.add_subplot(1, 1, 1)
    # # ax2 = fig.add_subplot(1, 2, 1)
    # # ax3 = fig.add_subplot(1, 3, 1)
    # # ys1 = []
    # # ys2 = []
    # # ys3 = []
    #
    # arduinoSerial.reset_input_buffer()
    #






    # Add x and y to lists

    # Limit x and y lists to 20 items
    # xs = xs[-20:]
    #
    #
    # # Draw x and y lists
    # ax.set_ylim([-60,60])
    # line1, = ax.plot(xs, ys)


    # plt.show()
    # while True:
    #     jsonString = arduinoSerial.readline().decode('utf-8')
    #     print(jsonString)
    #     jsonData = json.loads(jsonString)
    #     ys.append(jsonData['x_angle'])
    #
    #
    #     # Add x and y to lists
    #
    #     # Limit x and y lists to 20 items
    #     xs = xs[-20:]
    #     ys = ys[-20:]

        # Draw x and y lists


        # Format plot
        # plt.xticks(rotation=45, ha='right')
        # plt.subplots_adjust(bottom=0.30)
        # plt.title(' Time')
        # plt.ylabel('angle')
        # line1.set_ydata(np.asarray(ys))
        # fig.canvas.draw()
        # fig.canvas.flush_events()


