import time
import numpy as np
import serial as serial
from ctypes import *

from base import BaseComponent


LSB_CONNECT = b'\xB9'
LSB_MOTOR = b'\xA8'
ZERO = [0, 0, 0, 0, 0, 0]


class ServoInterface(BaseComponent):
    def __init__(self, name, publisher, **kwargs):
        super().__init__(name, publisher)
        timeout = kwargs.pop('timeout', 5)
        self.servo_directions = kwargs.pop('servo_directions')
        self.servo_indices = kwargs.pop('servo_indices')
        self.zero_positions = kwargs.pop('zero_positions')
        self.serial = serial.Serial(**kwargs)
        self.connect(timeout)

    def connect(self, timeout):
        t0 = time.time()
        while 'ACK' not in str(self.serial.read(3)):
            self.serial.write(LSB_CONNECT)
            for _ in range(6):
                self.serial.write(c_int8(0))
            self.serial.write(c_uint8(0))
            if time.time() - t0 > timeout:
                raise serial.SerialException('Unable to connect to servos within {} seconds'.format(timeout))

    def update(self, msg):
        if msg['type'] == 'servo_status':
            angles = msg['data']['servo_angles']
            # TODO: convert float angles (rad) to correct int outputs
            self.send_command(angles)
        elif msg['type'] == 'set_zero':
            self.send_command(ZERO)

    def send_command(self, angles):
        angles = self.format_angles(angles)
        self.serial.write(LSB_MOTOR)
        for i in range(6):
            self.serial.write(c_int8(angles[i]))
        self.serial.write(c_uint8(1))

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
        self.ready = False
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
            # print([a*180/np.pi for a in delta_ori])

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


# class DummyInterface(BaseComponent):
#     def __init__(self, name, publisher, **kwargs):
#         super().__init__(name, publisher)
#         sim_type = kwargs.get('sim_type', 'tra')
#         if sim_type == 'sax':
#             times, translations, orientations = self._single_axis_test()
#         elif sim_type == 'dax':
#             times, translations, orientations = self._dual_axis_test()
#         elif sim_type == 'tra':
#             times, translations, orientations = self._translation_test()
#         else:
#             raise ValueError('Innvalid simulation type: {}'.format(sim_type))
#         self.base_time = time.time()
#         self.pos_func = self._position_function(times, translations, orientations)
#
#     def consume(self, msg):
#         if msg['type'] == 'device_request':
#             translation, orientation = self.get_position()
#             self.publish(self.get_position_message(translation, orientation))
#
#     def get_position(self):
#         t = time.time() - self.base_time
#         return self.pos_func(t)
#
#     def get_position_message(self, translation, orientation):
#         msg = {
#             'sender': self.name,
#             'type': 'position',
#             'data': {
#                 'delta_translation': translation,
#                 'delta_orientation': orientation
#             }
#         }
#         return msg
#
#     @staticmethod
#     def _position_function(times, translations, orientations):
#         times = times - times[0]
#
#         def func(t):
#             if t > times[-1]:
#                 t = t%times[-1]
#             idx = np.where(times <= t)[0][-1]
#             return translations[idx], orientations[idx]
#
#         return func
#
#     @staticmethod
#     def _single_axis_test():
#         magnitude = np.pi/18
#         period = 4
#         axis = 0
#         n = 200
#         n = n - n%4
#         times = np.linspace(0, period, n)
#         translations = np.zeros((n, 3))
#         orientations = np.zeros((n, 3))
#         mags = np.linspace(0, magnitude, n//4)
#         orientations[:, axis] = np.concatenate((mags, np.flip(mags), -mags, np.flip(-mags)))
#         return times, translations, orientations
#
#     @staticmethod
#     def _dual_axis_test():
#         magnitudes = [np.pi/18, np.pi/6]
#         period = 6
#         axes = (0, 2)
#         n = 300
#         n = n - n%6
#         times = np.linspace(0, period, n)
#         translations = np.zeros((n, 3))
#         orientations = np.zeros((n, 3))
#         mags1 = np.linspace(0, magnitudes[0], n//12)
#         mags2 = np.linspace(0, magnitudes[1], n//12)
#         ori_1 = np.concatenate((mags1, np.ones(n//3)*mags1[-1], np.flip(mags1)))
#         ori_1 = np.concatenate((ori_1, -ori_1))
#         ori_2 = np.concatenate((np.zeros(n//12), mags2, np.flip(mags2),
#                                 -mags2, np.flip(-mags2), np.zeros(n//12)))
#         ori_2 = np.concatenate((ori_2, -ori_2))
#         orientations[:, axes[0]] = ori_1
#         orientations[:, axes[1]] = ori_2
#         return times, translations, orientations
#
#     @staticmethod
#     def _translation_test():
#         period = 60
#         a = 0.15
#         n = 400
#         times = np.linspace(0, period, n)
#         translations = np.zeros((n//2, 3))
#         orientations = np.zeros((n, 3))
#         translations[:, 0] = a*times[:n//2]*np.cos(times[:n//2])
#         translations[:, 1] = a*times[:n//2]*np.sin(times[:n//2])
#         translations = np.concatenate((translations, translations))
#         translations[n//2:, 0] = -translations[n//2:, 0]
#         translations[n//2:, :] = np.flip(translations[n//2:, :], axis=0)
#         return times, translations, orientations
