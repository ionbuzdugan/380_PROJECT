import time
import numpy as np
import serial as serial
import socket
from ctypes import *
from datetime import datetime as dtime

from base import BaseComponent

LSB_MOTOR = c_uint8(0)


class ServoInterface(BaseComponent):
    def __init__(self, name, publisher, **kwargs):
        startup_delay = kwargs.pop('startup_delay', 3)
        super().__init__(name, publisher)
        self.servo_directions = kwargs.pop('servo_directions')
        self.servo_indices = kwargs.pop('servo_indices')
        self.zero_positions = kwargs.pop('zero_positions')
        self.serial = serial.Serial(**kwargs)
        time.sleep(startup_delay)

    def update(self, msg):
        if msg['type'] == 'servo_status':
            angles = msg['data']['servo_angles']
            # TODO: convert float angles (rad) to correct int outputs
            self.send_command(angles)

    # def send_command(self, angles):
    #     angles = self.format_angles(angles)
    #     angles_bytes = bytearray(angles)
    #     self.serial.writelines(angles_bytes)
    #     for i in range(6):
    #         print('Sent: ', angles[i], ' Received: ', str(self.serial.readline()))

    def send_command(self, angles):
        # print('\nSending...')
        angles = self.format_angles(angles)
        for i in range(6):
            self.serial.write(c_uint8(angles[i]))
        # for i in range(6):
        #     print('Sent: ', angles[i], ' Received: ', str(self.serial.readline()))

    def format_angles(self, angles):
        formatted_angles = [0]*6
        for con_i, ser_i in enumerate(self.servo_indices):
            formatted_angles[ser_i] = int(round(180*(self.servo_directions[con_i] < 0)
                                                + self.servo_directions[con_i]*angles[con_i]*180/np.pi
                                                - self.servo_directions[con_i]*self.zero_positions[con_i]))
        return formatted_angles


class KeyboardInterface(BaseComponent):
    def __init__(self, name, publisher, master, **kwargs):
        super().__init__(name, publisher)
        self.master = master

        self.x_state = 0
        self.y_state = 0
        self.z_state = 0  # should remain so, as we are neglecting axial translation for now
        self.roll_state = 0
        self.pitch_state = 0
        self.yaw_state = 0  # should remain so, as we are neglecting axial rotation for now

        self.x_rate = kwargs['x_rate']
        self.y_rate = kwargs['y_rate']
        self.z_rate = kwargs['z_rate']
        self.roll_rate = kwargs['roll_rate']*np.pi/180
        self.pitch_rate = kwargs['pitch_rate']*np.pi/180
        self.yaw_rate = kwargs['yaw_rate']*np.pi/180

        self.prev_time = 0
        self.bind_handlers()

    def update(self, msg):
        if msg['type'] == 'update':
            new_time = time.time()
            d_x = self.x_state*(new_time - self.prev_time)*self.x_rate
            d_y = self.y_state*(new_time - self.prev_time)*self.y_rate
            d_z = self.z_state*(new_time - self.prev_time)*self.z_rate
            d_roll = self.roll_state*(new_time - self.prev_time)*self.roll_rate
            d_pitch = self.pitch_state*(new_time - self.prev_time)*self.pitch_rate
            d_yaw = self.yaw_state*(new_time - self.prev_time)*self.yaw_rate
            delta_trans = (d_x, d_y, d_z)
            delta_ori = (d_roll, d_pitch, d_yaw)
            self.prev_time = new_time
            self.publish({
                'sender': self.name,
                'type': 'delta',
                'data': {
                    'delta_translation': delta_trans,
                    'delta_orientation': delta_ori
                }
            })

    def bind_handlers(self):
        self.master.bind('<KeyPress>', self.keydown_handler)
        self.master.bind('<KeyRelease>', self.keyup_handler)

    def keydown_handler(self, event):
        if event.char == 'w':
            self.x_state = 1
        elif event.char == 'a':
            self.y_state = 1
        elif event.char == 's':
            self.x_state = -1
        elif event.char == 'd':
            self.y_state = -1
        elif event.char == '\uf700':
            self.pitch_state = 1
        elif event.char == '\uf702':
            self.roll_state = -1
        elif event.char == '\uf701':
            self.pitch_state = -1
        elif event.char == '\uf703':
            self.roll_state = 1

    def keyup_handler(self, event):
        if event.char == 'w':
            self.x_state = 0
        elif event.char == 'a':
            self.y_state = 0
        elif event.char == 's':
            self.x_state = 0
        elif event.char == 'd':
            self.y_state = 0
        elif event.char == '\uf700':
            self.pitch_state = 0
        elif event.char == '\uf702':
            self.roll_state = 0
        elif event.char == '\uf701':
            self.pitch_state = 0
        elif event.char == '\uf703':
            self.roll_state = 0


class IMUInterface(BaseComponent):
    def __init__(self, name, publisher, **kwargs):
        super().__init__(name, publisher)
        host = kwargs['host']
        port = kwargs['port']
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.bind((host, port))
        self.timeout = kwargs.get('timeout', 0.5)
        self.history = kwargs.get('history', 10)
        self.file_prefix = kwargs.get('file_prefix', '')
        self.min_resp = 0.5
        self.zero_pos = None
        self.t0 = None

        if self.file_prefix:
            ts = dtime.strftime(dtime.now(), '%Y-%m-%d_%H.%M.%S.csv')
            self.file = open(self.file_prefix + ts, 'w')
            self.file.write('{Time,Roll,Pitch,Yaw\n')

    def update(self, msg):
        if msg['type'] == 'update':
            if self.zero_pos is None:
                self.zero()
            reading = self.get_reading()
            if reading is not None:
                self.publish_reading(reading)

    def zero(self):
        reading = None
        while reading is None:
            reading = self.get_reading()
        self.zero_pos = reading
        self.t0 = time.time()

    def get_reading(self):
        readings = []
        while len(readings) < self.history:
            t0 = time.time()
            reading = None
            while time.time() - t0 < self.timeout:
                message, address = self.socket.recvfrom(8192)
                splt = message.decode('ascii').split(',')
                if len(splt) == 17:
                    reading = [float(n.strip()) for n in splt[-3:]]
                    break
            if reading is not None:
                readings.append(reading)
        reading = np.mean(readings, axis=0)
        reading = np.flip(reading)
        return reading

    def publish_reading(self, reading):
        reading_out = []
        for r, z in zip(reading, self.zero_pos):
            if abs(r - z) > self.min_resp:
                reading_out.append((r - z)*np.pi/180.)
            else:
                reading_out.append(0)

        reading_out[-1] = 0
        msg = {
            'sender': self.name,
            'type': 'imu_reading',
            'data': {
                'reading': reading_out,
            }
        }
        self.publish(msg)
        reading_out = [r*180/np.pi for r in reading_out]
        # print(reading_out[:2])
        if self.file_prefix:
            self.write(time.time() - self.t0, reading_out)

    def write(self, time, reading):
        self.file.write('{},{},{},{}'.format(time, reading[0], reading[1], reading[2]) + '\n')

    def __del__(self):
        self.file.close()
