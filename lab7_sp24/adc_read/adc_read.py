#!/usr/bin/env python

import time
import logging
import sys
import glob
import serial
import argparse

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from collections import deque

# Based on tutorial
# http://josephmr.com/realtime-arduino-sensor-monitoring-with-matplotlib/

# Python logging guidelines
# http://victorlin.me/posts/2012/08/26/good-logging-practice-in-python

MAX_X = 500   # Width of graph
MAX_Y = 1 # * 3.3 / 5  # Height of
xlims = np.arange(-MAX_X//2, MAX_X//2)


# Serial functions
def serial_ports():
    """Lists serial ports
    Raises:
    EnvironmentError:
        On unsupported or unknown platforms
    Returns:
        A list of available serial ports
    """
    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(256)]

    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this is to exclude your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')

    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.usbmodem*')

    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def open_port(port, baud):
    """Open a serial port.
    Args:
    port (string): port to open, on Unix typically begin with '/dev/tty', on
        or 'COM#' on Windows.
    baud (int, optional): baud rate for serial communication
    Raises:
    SerialException: raised for errors from serial communication
    Returns:
       A pySerial object to communicate with the serial port.
    """
    ser = serial.Serial()
    try:
        ser = serial.Serial(port, baud, timeout=1)
        logger.info("Opened serial connection on port %s"%port)
        ser.flushInput()
        a = ser.read(1)[0]
        a_is_high = a & 63
        if a_is_high:
            ser.read(1)
        return ser
    except serial.SerialException:
        raise

# Plotting functions
def update(fn, l2d, ser, line, data, num_read):
    # try:
    d = ser.read(num_read * 2)
    for i in range(num_read):
        base = i*2
        data[i] = (d[base] & 63) * 64 + (d[base+1] & 63)
        print(data[i])
    #logger.debug("Raw ADC value: %s"%data)

    # Add new point to deque
    line[:-num_read] = line[num_read:]
    # line[-num_read:] = data/1024 * 5 / 3.3 # this is used for 3.3V
    line[-num_read:] = data/1024 # this is used for 5V

    # Set the l2d to the new line coord ([x-coords], [y-coords])
    l2d.set_data(xlims, line)

def main(device="/dev/ttyACM0", baud=38400, plot=True, num_read=16):
    ser = serial.Serial()
    try:
        ser = open_port(device,baud)
    except:
        logger.error("Cannot open port %s"%device)
        available = serial_ports()
        if available:
            logger.info("The following ports are available: %s\n"%" "\
                    .join(available))
        else:
            logger.info("No serial ports detected.")
        quit()

    # Plot the values in real time
    if args.plot:
        fig = plt.figure()

        # make the axes revolve around [0,0] at the center
        # instead of the x-axis being 0 - +100, make it -50 - +50 ditto
        # for y-axis -512 - +512
        a = plt.axes(xlim=(-(MAX_X/2),MAX_X/2),ylim=(-0.1,MAX_Y + 0.1))
        # a = plt.axes(xlim=(-(MAX_X/2),MAX_X/2),ylim=(-(512),512))
        num_read = 16

        line = np.zeros(dtype=np.float32, shape=(MAX_X))
        data = np.zeros(dtype=np.float32, shape=(num_read))

        # plot an empty line and keep a reference to the line2d instance
        l1, = a.plot([], [])
        ani = anim.FuncAnimation(fig, update, fargs=(l1,ser, line, data, num_read), interval=0.001*num_read)
        # ani = anim.FuncAnimation(fig, update, fargs=(l1,ser, line, data, num_read), interval=num_read)
        plt.title("ADC output")
        plt.xlabel("t")
        plt.show()

    else:
        try:
            while True:
                _, _, a, b = ser.read(4)
                logger.debug(int(a) * 256 + int(b))
        except serial.SerialTimeoutException:
            logger.info("Device disconnected")
            quit()
        except serial.SerialException:
            raise
        except Exception as e:
            logger.error('', exc_info=True)

if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser(description='Record ADC output.')
    parser.add_argument('-D', '--device', choices=serial_ports(), default="/dev/ttyACM0",
            help='a serial port to read from')
    '''
        In spring 2018 we attempted changing the baud rate to 38400 to match
        the rest of the project, but it did not work.
        KEEP AT 9600.
    '''
    parser.add_argument('-b', '--baud', type=int, default=9600,
            help="baud rate for the serial connection ")
    parser.add_argument('-p', '--plot', action='store_true', default=True,
            help='show real time plot (requires X11 to be enabled)')

    parser.add_argument('--debug', action='store_true', default=False,
            help='print all raw ADC values')

    parser.add_argument('-n', '--nsimult', default=16, help='Number of cycles to process at a time', type=int)
    args = parser.parse_args()

    if args.debug:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    main(args.device, args.baud, args.plot, args.nsimult)
