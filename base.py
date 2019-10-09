import tkinter as tk


class BaseComponent:
    def __init__(self, name, publisher, **kwargs):
        self.name = name
        self.publisher = publisher

    def publish(self, msg):
        self.publisher.publish(msg)

    def update(self, msg):
        raise NotImplementedError


class BaseFrame(BaseComponent):
    def __init__(self, name, publisher, master, **kwargs):
        super().__init__(name, publisher)
        grid_params = kwargs.pop('grid_params')
        self.width = kwargs['width']
        self.height = kwargs['height']
        self.frame = tk.Frame(master, highlightbackground='black', highlightcolor='black',
                              highlightthickness=1, borderwidth=0, **kwargs)
        self.frame.grid(**grid_params)

    def update(self, msg):
        raise NotImplementedError
