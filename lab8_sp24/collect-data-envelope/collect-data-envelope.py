# collect-data-envelope.py
# Python script for data collection in SIXT33N Voice Classification Lab
# 
# EE16B Spring 2016
# Emily Naviasky & Nathaniel Mailoa
#
# EE 16B Spring 2024
# Venkata Alapati

import serial
import sys
import os
import re
import glob

samples = []

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
        ser = serial.Serial(port, baud, timeout=10)
        print("Opened serial connection on port %s"%port)
        return ser
    except serial.SerialException:
        raise

def save_txt(buff, txtfile):
  text_file = open(txtfile, 'a')
  text_file.write(','.join([str(x) for x in buff]))
  text_file.write('\n')
  text_file.close()

def run(filename, sampleLen=45):
  if not filename.endswith('.csv'):
    print("Warning: filename does not end with .csv. Appending .csv to the filename.")
    filename = filename + '.csv'

  samples = []
  print("EE16B Lab 8 Data Collection")

  ports = serial_ports()
  if ports:
    print("Available serial ports:")
    for (i,p) in enumerate(ports):
      print("%d) %s"%(i+1,p))
  else:
    print("No ports available. Check serial connection and try again.")
    print("Exiting...")
    return

  portNo = input("Select the port to use: ")
  ser = serial.Serial(ports[int(portNo)-1])
  ser.baudrate=38400
  ser.timeout=20
  
  print('Collecting data... Please wait.')
  
  curr = 0

  while(curr < sampleLen):
    samples = []
    while (not re.match("Start", ser.readline().decode())):
      pass
    for i in range(172):
      samples.append((float)(ser.readline().decode().rstrip('\n')))
    save_txt(samples, filename)
    print("Done writing sample " + str(curr+1) + " to " + filename)
    curr +=1 

  print('Data collection complete.')

  ser.close()


if __name__ == "__main__":
    if(len(sys.argv) != 2):
        print("Usage: python collect-data.py <filename.csv>")
        print("You likely forgot to include the filename argument.")
        print("Exiting...")
        exit()
    run(sys.argv[1])
    