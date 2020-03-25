#!/usr/bin/env python3
import time
import serial
import re
import threading 
from parse import *



class Communicator(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.ser = serial.Serial(
         port='/dev/ttyUSB0',
         baudrate = 115200,
         #parity=serial.PARITY_NONE,
         #stopbits=serial.STOPBITS_ONE,
         #bytesize=serial.EIGHTBITS,
         #timeout=1
        )
        self.__data = dict()
        self.__data_lock = threading.Condition()
    
    def run(self):
        while 1:
            try:
                serial_data = re.sub('\r\n', '', self.ser.readline().decode())
                parsed_str = parse('{} {} {} {} {} {} {} {} {} {}', serial_data)
            except KeyboardInterrupt:
                break;
            except:
                continue;

            if parsed_str is None:
                continue

            self.__data_lock.acquire()
            self.__data['V']   = float(parsed_str[0])
            self.__data['kp']  = float(parsed_str[1])
            self.__data['kd']  = float(parsed_str[2])
            self.__data['ki']  = float(parsed_str[3])
            self.__data['e']   = float(parsed_str[4])
            self.__data['de']  = float(parsed_str[5])
            self.__data['ie']  = float(parsed_str[6])
            self.__data['cmd'] = float(parsed_str[7])
            self.__data['vl']  = float(parsed_str[8])
            self.__data['vr']  = float(parsed_str[9])
            self.__data_lock.notifyAll()
            self.__data_lock.release()

    def data(self):
        self.__data_lock.acquire()
        if self.__data_lock.wait(0.1):
            return self.__data.copy()
        else:
            return None


comm = Communicator()
comm.start()


while True:
    print(comm.data());

