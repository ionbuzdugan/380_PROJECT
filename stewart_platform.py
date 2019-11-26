import yaml
import time

from base import BaseComponent
from publisher import Publisher
from kinematics import KinematicsController
from gui import GUI, SimulationFrame, GraphFrame
from device_interfaces import KeyboardInterface, ServoInterface


def get_config(config_file):
    with open(config_file, 'r') as yf:
        config = yaml.safe_load(yf)
    return config


class Orchestrator(BaseComponent):
    def __init__(self, config_file):
        super().__init__('orchestrator', Publisher())
        config = get_config(config_file)
        self.gui = GUI()
        self.components = {}
        self.consumers = {}
        self.loop_delay = config['loop_delay']
        self.install_components(config['components'])

    def install_components(self, components):
        for comp in components:
            name = comp.pop('name')
            cls = comp.pop('class')
            consumers = comp.pop('consumers', [])
            if name not in self.components:
                if cls == 'simulation_frame':
                    self.components[name] = SimulationFrame(name, self.publisher, self.gui, **comp)
                elif cls == 'graph_frame':
                    self.components[name] = GraphFrame(name, self.publisher, self.gui, **comp)
                elif cls == 'kinematics_controller':
                    self.components[name] = KinematicsController(name, self.publisher, **comp)
                elif cls == 'keyboard_device_interface':
                    self.components[name] = KeyboardInterface(name, self.publisher, self.gui, **comp)
                elif cls == 'servo_device_interface':
                    self.components[name] = ServoInterface(name, self.publisher, **comp)
                else:
                    raise ValueError('Invalid component type: {}'.format(cls))
            else:
                raise ValueError('Multiple components provided with name {}'.format(name))
            self.publisher.consumers[name] = consumers
        for name, consumers in self.publisher.consumers.items():
            self.publisher.consumers[name] = [self.components[c] for c in consumers]
            # self.publisher.consumers[name].append(self)
        self.publisher.consumers[self.name] = [comp for comp in self.components.values()]

    def update(self, msg=None):
        self.publish({
            'sender': self.name,
            'type': 'update'
        })
        self.gui.update()
        time.sleep(self.loop_delay)

    def run(self):
        while True:
            self.update()


if __name__ == '__main__':
    conf = 'stewart_config.yml'
    app = Orchestrator(conf)
    app.run()
