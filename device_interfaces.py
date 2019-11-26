import time
import numpy as np
import serial as serial
from ctypes import *

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

    def send_command(self, angles):
        # print([a*180/np.pi for a in angles])
        angles = self.format_angles(angles)
        angles_bytes = bytearray([0] + angles)
        self.serial.write(angles_bytes)
        for i in range(6):
            print('Sent: ', angles[i], ' Received: ', str(self.serial.readline()))

    # def send_command(self, angles):
    #     print([a*180/np.pi for a in angles])
    #     angles = self.format_angles(angles)
    #     # angles = bytearray(angles)
    #     print('\nSending...')
    #     # print(angles)
    #     self.serial.write(LSB_MOTOR)
    #     for i in range(6):
    #         self.serial.write(c_uint8(angles[i]))
    #     for i in range(6):
    #         print('Sent: ', angles[i], ' Received: ', str(self.serial.readline()))

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
