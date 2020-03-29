#!/usr/bin/env python3
import time
import serial
import re
import threading
import json
from parse import *

from quart import Quart, websocket, render_template, request


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
        self.__serial_lock = threading.Lock()

        self.__data['error'] = ''

    def run(self):
        while 1:
            try:
                self.__serial_lock.acquire()
                serial_data = re.sub('\r\n', '', self.ser.readline().decode())
                parsed_str = parse('{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}', serial_data)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print("Failed to read from Serial: {}".format(e))
                continue;
            finally:
                self.__serial_lock.release()

            if parsed_str is None:
                continue

            self.__data_lock.acquire()
            self.__data['V']     = float(parsed_str[0])
            self.__data['i_kp']  = float(parsed_str[1])
            self.__data['i_kd']  = float(parsed_str[2])
            self.__data['i_ki']  = float(parsed_str[3])
            self.__data['i_e']   = float(parsed_str[4])/10
            self.__data['i_de']  = float(parsed_str[5])/10
            self.__data['i_ie']  = float(parsed_str[6])/10
            self.__data['cmd']   = float(parsed_str[7])
            self.__data['vl']    = float(parsed_str[8])
            self.__data['vr']    = float(parsed_str[9])
            self.__data['o_kp']  = float(parsed_str[10])
            self.__data['o_kd']  = float(parsed_str[11])
            self.__data['o_ki']  = float(parsed_str[12])
            self.__data['o_e']   = float(parsed_str[13])/10
            self.__data['o_de']  = float(parsed_str[14])/10
            self.__data['o_ie']  = float(parsed_str[15])/10
            self.__data['a_ref'] = float(parsed_str[16])/10

            self.__data_lock.notifyAll()
            self.__data_lock.release()

    def data(self):
        self.__data_lock.acquire()
        if self.__data_lock.wait(0.15):
            return_data = self.__data.copy()

            if self.__data['V'] < 11.4:
                return_data['error'] = "Undervoltage!"
        else:
            return_data = None

        self.__data_lock.release()

        return return_data

    def lastData(self):
        self.__data_lock.acquire()
        data_copy = self.__data.copy()
        self.__data_lock.release()

        return data_copy

    def update_inner(self, kp, kd, ki):
        try:
            self.__serial_lock.acquire()
            print("Requested param update to: kp: {} kd: {} ki: {}".format(kp, kd, ki))
            self.ser.write('i/{}/{}/{}/\r\n'.format(kp, kd, ki).encode())
        except Exception as e:
            print("Failed to write to serial: {}".format(e))
        finally:
            self.__serial_lock.release()

    def update_outer(self, kp, kd, ki):
        try:
            self.__serial_lock.acquire()
            print("Requested param update to: kp: {} kd: {} ki: {}".format(kp, kd, ki))
            self.ser.write('o/{}/{}/{}/\r\n'.format(kp, kd, ki).encode())
        except Exception as e:
            print("Failed to write to serial: {}".format(e))
        finally:
            self.__serial_lock.release()


app = Quart(__name__)
comm = Communicator()


@app.websocket('/ws')
async def ws():
    while True:
        data = comm.data()
        if data is not None:
            await websocket.send(json.dumps(data))
        else:
            fail_data = comm.lastData()
            fail_data['error'] = 'No data received from HW.'
            await websocket.send(json.dumps(fail_data))


@app.route('/')
async def index():
    return await render_template("index.html")

@app.route('/set_inner_params', methods=['POST'])
async def updateInnerParams():
    form_data = await request.data
    form_str = form_data.decode()

    try:
        form = json.loads(form_str)
        kp = form['kp']
        kd = form['kd']
        ki = form['ki']
        comm.update_inner(kp, kd, ki)
    except:
        print("Failed to parse {} as json".format(form_str))

    return await render_template("index.html")


@app.route('/set_outer_params', methods=['POST'])
async def updateOuterParams():
    form_data = await request.data
    form_str = form_data.decode()

    try:
        form = json.loads(form_str)
        kp = form['kp']
        kd = form['kd']
        ki = form['ki']
        comm.update_outer(kp, kd, ki)
    except Exception as e:
        print("Failed to parse {} as json: {}".format(form_str, e))

    return await render_template("index.html")


if __name__ == "__main__":
    comm.start()
    app.run(host='0.0.0.0', port=8080)
