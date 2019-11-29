import numpy as np

from base import BaseComponent


MIN_ANGLE = -np.pi/2  # minimum servo angle
MAX_ANGLE = np.pi/2  # maximum servo angle


def get_rotation_matrix(roll, pitch, yaw):
    rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    r = np.matmul(rz, np.matmul(ry, rx))
    return r


class KinematicsController(BaseComponent):
    def __init__(self, name, publisher, **kwargs):
        super().__init__(name, publisher)

        # Construction parameters
        self.rod_length = kwargs['rod_length']
        self.crank_length = kwargs['crank_length']
        self.actuators = kwargs['actuators']

        # Motion Parameters
        self.servo_angles = None
        self.servo_range = (max(kwargs['servo_range'][0], MIN_ANGLE*180/np.pi),
                            max(kwargs['servo_range'][1], MAX_ANGLE*180/np.pi))

        # Vector variables
        self.translation = None  # current translation
        self.orientation = None  # current orientation
        self.B = []  # base joints in base frame
        self.P = []  # platform joints in platform frame
        self.q = []  # vector from base origin to P
        self.l = []  # vector from B to P
        self.H = []  # servo horn end to mount the rod
        self.sin_beta = []  # sin of pan angle of servos in base plate
        self.cos_beta = []  # cos of pan angle of servos in base plate
        for act in self.actuators:
            self.B.append(act['base_joint'])
            self.P.append(act['platform_joint'])
            self.sin_beta.append(np.sin(act['crank_angle']))
            self.cos_beta.append(np.cos(act['crank_angle']))
            self.q.append([0, 0, 0])
            self.l.append([0, 0, 0])
            self.H.append([0, 0, 0])
        self.T0 = [0, 0, np.sqrt(self.rod_length**2 + self.crank_length**2
                                 - (self.P[0][0] - self.B[0][0])**2
                                 - (self.P[0][1] - self.B[0][1])**2)]

        self.update_platform([0, 0, 0], [0, 0, 0])

    def update(self, msg):
        if msg['type'] in ('position', 'delta', 'imu_reading'):
            if msg['type'] == 'position':
                translation = msg['data']['translation']
                orientation = msg['data']['orientation']
            elif msg['type'] == 'delta':
                translation = []
                orientation = []
                for i in range(3):
                    translation.append(self.translation[i] + msg['data']['delta_translation'][i])
                    orientation.append(self.orientation[i] + msg['data']['delta_orientation'][i])
            elif msg['type'] == 'imu_reading':
                roll, pitch = msg['data']['reading'][:2]
                max_ang = 15*np.pi/180
                orientation = [0, 0, 0]
                orientation[0] = np.sign(roll)*min(np.abs(roll), max_ang)
                orientation[1] = -np.sign(pitch)*min(np.abs(pitch), max_ang)
                translation = self.translation
            self.update_platform(translation, orientation)
            self.publish_servo()
            self.publish_geometry()

    def publish_geometry(self):
        cranks, rods = [], []
        for i in range(6):
            cranks.append([self.B[i], self.H[i]])
            rods.append([self.H[i], self.q[i]])
        msg = {
            'sender': self.name,
            'type': 'platform_geometry',
            'data': {
                'base_pts': np.asarray(self.B),
                'platform_pts': np.asarray(self.q),
                'cranks': np.asarray(cranks),
                'rods': np.asarray(rods)
            }
        }
        self.publish(msg)

    def publish_servo(self):
        msg = {
            'sender': self.name,
            'type': 'servo_status',
            'data': {
                'servo_angles': self.servo_angles,
                'servo_range': self.servo_range
            }
        }
        self.publish(msg)

    def update_platform(self, translation, orientation):
        rot_matrix = get_rotation_matrix(orientation[0], orientation[1], orientation[2])
        q, l, H = [], [], []
        servo_angles = []
        for i in range(6):
            q.append([0, 0, 0])
            l.append([0, 0, 0])
            H.append([0, 0, 0])

            Bi = self.B[i]

            o = np.matmul(rot_matrix, np.reshape(self.P[i], (3, 1))).ravel()

            q[i][0] = translation[0] + o[0]
            q[i][1] = translation[1] + o[1]
            q[i][2] = translation[2] + o[2] + self.T0[2]

            l[i][0] = q[i][0] - Bi[0]
            l[i][1] = q[i][1] - Bi[1]
            l[i][2] = q[i][2] - Bi[2]

            gk = l[i][0]**2 + l[i][1]**2 + l[i][2]**2 - self.rod_length**2 + self.crank_length**2
            ek = 2*self.crank_length*l[i][2]
            fk = 2*self.crank_length*(self.cos_beta[i]*l[i][0] + self.sin_beta[i]*l[i][1])

            sq_sum = ek**2 + fk**2

            # Check validity of position before continuing
            gk2_sq = gk**2/sq_sum
            if gk2_sq > 1:
                return

            sqrt1 = np.sqrt(1 - gk2_sq)
            sqrt2 = np.sqrt(sq_sum)

            sin_alpha = (gk*ek)/sq_sum - (fk*sqrt1)/sqrt2
            cos_alpha = (gk*fk)/sq_sum + (ek*sqrt1)/sqrt2

            H[i][0] = Bi[0] + self.crank_length*cos_alpha*self.cos_beta[i]
            H[i][1] = Bi[1] + self.crank_length*cos_alpha*self.sin_beta[i]
            H[i][2] = Bi[2] + self.crank_length*sin_alpha

            # Check angle validity
            servo_angles.append(np.arcsin((H[i][2] - Bi[2])/self.crank_length))
            if np.isnan(servo_angles[i]) or not (self.servo_range[0] < servo_angles[i] < self.servo_range[1]):
                return

        self.servo_angles = servo_angles
        self.translation = translation
        self.orientation = orientation
        self.q = q
        self.l = l
        self.H = H
