import time
import asyncio
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, art3d
from matplotlib import cm
import tkinter as tk
from PIL import Image, ImageTk

from base import BaseFrame


ROD_COLORS = cm.get_cmap('jet')(np.linspace(0, 1., 6))[[0, 3, 1, 4, 2, 5], :3]


class GUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Stewart Platform Interface')
        self.geometry = '{}x{}+0+0'.format(self.winfo_screenwidth(), self.winfo_screenheight())
        self.lift()
        self.attributes('-topmost', True)
        self.after_idle(self.attributes, '-topmost', False)
        self.resizable(0, 0)


class SimulationFrame(BaseFrame):
    def __init__(self, name, publisher, master, **kwargs):
        self.limits = kwargs.pop('bbox_limits')
        self.pad = kwargs.pop('pad')
        self.fig = plt.figure()
        self.fig.dpi = kwargs.pop('dpi', 200)
        self.ax = self.fig.add_subplot(111, projection='3d')

        super().__init__(name, publisher, master, **kwargs)

        self.canvas = tk.Canvas(self.frame, width=self.width, height=self.height,
                                highlightthickness=0, borderwidth=0)
        self.canvas.grid()

        self.image = None
        self.crop = None
        self.image_id = self.canvas.create_image(self.width//2, self.height//2,
                                                 image=self.image, anchor='center')

    def update(self, msg):
        if msg['type'] == 'platform_geometry':
            geometry = msg['data']

            base_pts = geometry['base_pts']
            platform_pts = geometry['platform_pts']
            cranks = geometry['cranks']
            rods = geometry['rods']

            self.ax.clear()
            self.ax.view_init(elev=10, azim=180)
            self.ax._axis3don = False
            self.plot_poly3d(base_pts, 'k')
            self.plot_poly3d(platform_pts, 'g')
            self.plot_actuators(cranks, rods)
            self.plot_bbox()
            self.update_image()

    def update_image(self):
        self.fig.canvas.draw()
        buf = self.fig.canvas.tostring_rgb()
        ncols, nrows = self.fig.canvas.get_width_height()
        img = np.fromstring(buf, dtype=np.uint8).reshape((nrows, ncols, 3))
        if self.crop is None:
            self.get_crop(img)
        img = img[self.crop[0]:self.crop[1], self.crop[2]:self.crop[3], :]
        img = Image.fromarray(img)
        scale = min(self.width/img.size[0], self.height/img.size[1])
        new_size = (int(scale*img.size[0]), int(scale*img.size[1]))
        img = img.resize(new_size, Image.BICUBIC)
        self.image = ImageTk.PhotoImage(image=img)
        self.canvas.itemconfig(self.image_id, image=self.image)

    def get_crop(self, img):
        r_idx = np.where(img.mean(axis=1) < 255)[0]
        c_idx = np.where(img.mean(axis=0) < 255)[0]
        min_r = max(0, r_idx[0] - self.pad[0])
        max_r = min(img.shape[0], r_idx[-1] + self.pad[1])
        min_c = max(0, c_idx[0] - self.pad[2])
        max_c = min(img.shape[1], c_idx[-1] + self.pad[3])
        self.crop = (min_r, max_r, min_c, max_c)

    def plot_poly3d(self, pts, color='k', alpha=0.5):
        poly3d = art3d.Poly3DCollection([pts], color=color, edgecolors='k', linewidth=1, alpha=alpha)
        self.ax.add_collection3d(poly3d)

    def plot_actuators(self, cranks, rods):
        for i in range(6):
            self.ax.plot(cranks[i, :, 0], cranks[i, :, 1], cranks[i, :, 2], 'r-', linewidth=2)
            self.ax.plot(rods[i, :, 0], rods[i, :, 1], rods[i, :, 2], color=ROD_COLORS[i], linewidth=2)

    def plot_bbox(self):
        max_range = np.array([self.limits[0][1] - self.limits[0][0],
                              self.limits[1][1] - self.limits[1][0], self.limits[2][1] - self.limits[2][0]]).max()
        xb = 0.5*max_range*np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5*(self.limits[0][0] + self.limits[0][1])
        yb = 0.5*max_range*np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5*(self.limits[1][0] + self.limits[1][1])
        zb = 0.5*max_range*np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5*(self.limits[2][0] + self.limits[2][1])
        for xb, yb, zb in zip(xb, yb, zb):
            self.ax.plot([xb], [yb], [zb], 'w')


class GraphFrame(BaseFrame):
    def __init__(self, name, publisher, master, mode='angle', title='', xlabel='', ylabel='', keep_time=10, **kwargs):
        super().__init__(name, publisher, master, **kwargs)
        self.fig, self.ax = plt.subplots()
        self.plots = [self.ax.plot([0], [0], color=ROD_COLORS[i], linewidth=1)[0] for i in range(6)]

        self.mode = mode
        self.ax.set_title(title)
        self.ax.set_xlabel(xlabel)
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlim(-keep_time, 0)
        width_inches = self.width/self.fig.dpi
        height_inches = self.height/self.fig.dpi
        self.fig.set_size_inches(width_inches, height_inches)
        self.fig.subplots_adjust(left=0.125, right=0.9, bottom=0.15, top=0.9, wspace=0.2, hspace=0.2)

        self.keep_time = keep_time
        self.xdata = []
        self.ydata = []

        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.get_tk_widget().grid()

    def update(self, msg):
        if msg['type'] == 'servo_status' and self.mode in ('angle', 'speed'):
            self.xdata.append(time.time())
            if self.mode == 'angle':
                self.ydata.append([a*180/np.pi for a in msg['data']['servo_angles']])
                self.ax.set_ylim(msg['data']['servo_range'][0], msg['data']['servo_range'][1])
            elif self.mode == 'speed':
                self.ydata.append(msg['data']['servo_speeds'])
                self.ax.set_ylim(-msg['data']['max_speed'], msg['data']['max_speed'])
            time_delta = np.asarray(self.xdata) - self.xdata[-1]
            start_idx = np.where(time_delta > -1.5*self.keep_time)[0][0]
            if start_idx:
                del self.xdata[:start_idx]
                del self.ydata[:start_idx]
            ydata = np.asarray(self.ydata)
            for i in range(6):
                self.plots[i].set_data(time_delta[start_idx:], ydata[:, i])
            self.canvas.draw()


# class GraphTest:
#     figure_canvas_agg = FigureCanvasAgg(figure)
#     figure_canvas_agg.draw()
#     figure_x, figure_y, figure_w, figure_h = figure.bbox.bounds
#     figure_w, figure_h = int(figure_w), int(figure_h)
#     photo = tk.PhotoImage(master=canvas, width=figure_w, height=figure_h)
#
#     # Position: convert from top-left anchor to center anchor
#     canvas.create_image(loc[0] + figure_w/2, loc[1] + figure_h/2, image=photo)
#
#     # Unfortunately, there's no accessor for the pointer to the native renderer
#     tkagg.blit(photo, figure_canvas_agg.get_renderer()._renderer, colormode=2)
#
#     # Return a handle which contains a reference to the photo object
#     # which must be kept live or else the picture disappears
#     return photo

# class SimulationFrame2(BaseFrame):
#     def __init__(self, name, master, **kwargs):
#         self.limits = kwargs.pop('bbox_limits')
#         super().__init__(name, master, **kwargs)
#         self.fig = plt.figure()
#         self.ax = self.fig.add_subplot(111, projection='3d')
#         self.ax.view_init(elev=10, azim=180)
#         width_inches = self.width/self.fig.dpi
#         height_inches = self.height/self.fig.dpi
#         self.fig.set_size_inches(width_inches, height_inches)
#         self.fig.canvas.draw()
#         self.background = self.fig.canvas.copy_from_bbox(self.ax.bbox)
#         # self.fig.subplots_adjust(left=0., right=0.9, bottom=0.15, top=0.9, wspace=0.2, hspace=0.2)
#
#         self.canvas = FigureCanvasTkAgg(self.fig, self)
#         self.canvas.get_tk_widget().grid()
#
#     def update(self, msg):
#         if msg['type'] == 'platform_geometry':
#             geometry = msg['data']
#
#             base_pts = geometry['base_pts']
#             platform_pts = geometry['platform_pts']
#             cranks = geometry['cranks']
#             rods = geometry['rods']
#
#             self.ax.clear()
#             self.ax._axis3don = False
#
#             self.fig.canvas.restore_region(self.background)
#             self.plot_poly3d(base_pts, 'k')
#             self.plot_poly3d(platform_pts, 'g')
#             self.plot_actuators(cranks, rods)
#             self.plot_bbox()
#             self.fig.canvas.blit(self.ax.bbox)
#             self.canvas.draw()
#
#     def plot_poly3d(self, pts, color='k', alpha=0.5):
#         poly3d = art3d.Poly3DCollection([pts], color=color, edgecolors='k', linewidth=1, alpha=alpha)
#         self.ax.add_collection3d(poly3d)
#
#     def plot_actuators(self, cranks, rods):
#         for i in range(6):
#             pc = self.ax.plot(cranks[i, :, 0], cranks[i, :, 1], cranks[i, :, 2], 'r-', linewidth=2)[0]
#             pr = self.ax.plot(rods[i, :, 0], rods[i, :, 1], rods[i, :, 2], color=ROD_COLORS[i], linewidth=2)[0]
#             self.ax.draw_artist(pc)
#             self.ax.draw_artist(pr)
#
#     def plot_bbox(self):
#         max_range = np.array([self.limits[0][1] - self.limits[0][0],
#                               self.limits[1][1] - self.limits[1][0], self.limits[2][1] - self.limits[2][0]]).max()
#         xb = 0.5*max_range*np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5*(self.limits[0][0] + self.limits[0][1])
#         yb = 0.5*max_range*np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5*(self.limits[1][0] + self.limits[1][1])
#         zb = 0.5*max_range*np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5*(self.limits[2][0] + self.limits[2][1])
#         for xb, yb, zb in zip(xb, yb, zb):
#             b = self.ax.plot([xb], [yb], [zb], 'w')[0]
#             self.ax.draw_artist(b)