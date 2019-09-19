import sys
import time

import numpy as np
from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import (FigureCanvasQTAgg as FigureCanvas,
                                                NavigationToolbar2QT as NavigationToolbar)
from matplotlib.backend_bases import Event
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation

import multiprocessing as mp
import threading


if sys.platform != 'win32':
    from pyky040 import Encoder
    win32 = False
else:
    win32 = True


class ExpandingHeightButton(QtWidgets.QPushButton):
    def __init__(self, *argv):
        super().__init__(*argv)
        # Overload init to force fixed size
        self.setMaximumWidth(100)
        self.setSizePolicy(QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Expanding))


class NavToolbarForceUpdate(NavigationToolbar):
    def __init__(self, *argv):
        super().__init__(*argv)

    # Overload history buttons update function to add callback for drawing while stopped
    def set_history_buttons(self):
        can_backward = self._nav_stack._pos > 0
        can_forward = self._nav_stack._pos < len(self._nav_stack._elements) - 1
        self._actions['back'].setEnabled(can_backward)
        self._actions['forward'].setEnabled(can_forward)
        event = Event('history', self)
        self.canvas.callbacks.process('history', event)


class ApplicationWindow(QtWidgets.QMainWindow):
    # Currently, the home/back/forward + pan/zoom methods are laggy because they are running the old clear/draw
    # cycle in the background due to tight integration. External button inputs run much smoother as they only rely
    # on newer/faster drawing methods. Removing unnecessary draw calls would significantly speed up the process.

    def __init__(self):
        super().__init__()
        # stop/run variable
        self.stop = False

        # x limits
        self.x_lim = [0, 10]
        self.x_lim_manual = False

        # y limits
        self.y_lim = [0, 10]
        self.y_lim_manual = False

        # Create window and main widget
        self._main = QtWidgets.QWidget()
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.setFixedSize(800, 480)
        self.setCentralWidget(self._main)
        layout = QtWidgets.QHBoxLayout(self._main)

        # Create canvas and add toolbar to bottom
        # (Toolbar on L/R side is buggy and doesn't show cursor position...)
        dynamic_canvas = FigureCanvas(Figure())
        layout.addWidget(dynamic_canvas)
        self.nav_toolbar = NavToolbarForceUpdate(dynamic_canvas, self)
        plot_toolbar = self.addToolBar(QtCore.Qt.BottomToolBarArea, self.nav_toolbar)

        self.t = np.linspace(-15, 15, 101)
        self.y = np.sin(self.t + time.time())

        self.data_index = 0
        self.x = np.full(1000000, np.nan)
        self.z = np.full(1000000, np.nan)

        # Set up subplots and axes -- including update timer
        self._dynamic_ax = dynamic_canvas.figure.subplots()
        # self._timer = dynamic_canvas.new_timer(
        #     400, [(self._update_canvas, (), {})])
        # self._timer.start()
        self._dynamic_ax.autoscale_view(False, False, False)
        self.ln, = self._dynamic_ax.plot([], [], 'g.', scalex=False, scaley=False)


        # Create buttons
        buttons_layout = QtWidgets.QVBoxLayout()

        self.button_a = ExpandingHeightButton('Stop')
        self.button_b = ExpandingHeightButton('Run')
        self.button_c = ExpandingHeightButton('C')
        self.button_d = ExpandingHeightButton('D')
        self.button_e = ExpandingHeightButton('Exit')

        self.info_label = QtWidgets.QLabel("1\n2\n3\n4...")
        self.info_label.setFixedWidth(100)

        # Add buttons to layout
        buttons_layout.addWidget(self.button_a)
        buttons_layout.addWidget(self.button_b)
        buttons_layout.addWidget(self.button_c)
        buttons_layout.addWidget(self.button_d)
        buttons_layout.addWidget(self.button_e)
        buttons_layout.addWidget(self.info_label)

        # Connect buttons to functions
        self.button_a.clicked.connect(self.button_a_clicked)
        self.button_b.clicked.connect(self.button_b_clicked)
        self.button_c.clicked.connect(self.button_c_clicked)
        self.button_d.clicked.connect(self.button_d_clicked)
        self.button_e.clicked.connect(self.button_e_clicked)

        layout.addLayout(buttons_layout)

        # Events
        self.cid = self._dynamic_ax.figure.canvas.mpl_connect('button_press_event', self.onclick)
        self.move_mouse = self._dynamic_ax.figure.canvas.mpl_connect('motion_notify_event', self.mouse)
        # self.button_press = self._dynamic_ax.figure.canvas.mpl_connect('button_press_event', self.mouse)
        self.button_release = self._dynamic_ax.figure.canvas.mpl_connect('button_release_event', self.mouse)
        self.history = self._dynamic_ax.figure.canvas.mpl_connect('history', self.update_once)

        # Animate the canvas
        self.ani = FuncAnimation(self._dynamic_ax.figure, self._update_canvas, blit=True, interval=1)

        if not win32:
            # Initialise encoder threads
            self.h_pos_encoder = Encoder(CLK=11, DT=7)
            self.h_pos_encoder.setup(scale_min=-1e8, scale_max=1e8, step=1, inc_callback=self.pan_right,
                                     dec_callback=self.pan_left)
            self.h_pos_encoder_thread = threading.Thread(target=self.h_pos_encoder.watch)

            self.h_zoom_encoder = Encoder(CLK=13, DT=15)
            self.h_zoom_encoder.setup(scale_min=-1e8, scale_max=1e8, step=1, inc_callback=self.horizontal_zoom_in,
                                      dec_callback=self.horizontal_zoom_out)
            self.h_zoom_encoder_thread = threading.Thread(target=self.h_zoom_encoder.watch)

            self.v_pos_encoder = Encoder(CLK=12, DT=16)
            self.v_pos_encoder.setup(scale_min=-1e8, scale_max=1e8, step=1, inc_callback=self.pan_up,
                                     dec_callback=self.pan_down)
            self.v_pos_encoder_thread = threading.Thread(target=self.v_pos_encoder.watch)

            self.v_zoom_encoder = Encoder(CLK=18, DT=22)
            self.v_zoom_encoder.setup(scale_min=-1e8, scale_max=1e8, step=1, inc_callback=self.vertical_zoom_in,
                                      dec_callback=self.vertical_zoom_out)
            self.v_zoom_encoder_thread = threading.Thread(target=self.v_zoom_encoder.watch)

        # Set up  pipes
        self.plot_pipe_t, self.plot_pipe_p = mp.Pipe()

        # Timer for callbacks for copy data into arrays
        # self.teensy_timer = dynamic_canvas.new_timer(interval=100)
        # self.teensy_timer.add_callback(self.data_copy_callback)
        # self.teensy_timer.start()
        # Thread for callbacks instead? What is better?
        self.data_thread = threading.Thread(target=self.data_copy_callback, args=())
        self.data_thread.daemon = True
        self.data_thread.start()

        # Actual Teensy receiver side
        self.teensy = TeensyReceiver(self.plot_pipe_t)
        self.teensy_process = mp.Process(target=self.teensy, args=())
        self.teensy_process.daemon = True
        self.teensy_process.start()

    def encoders_start(self):
        # Start the encoders
        self.h_pos_encoder_thread.daemon = True
        self.h_pos_encoder_thread.start()

        self.h_zoom_encoder_thread.daemon = True
        self.h_zoom_encoder_thread.start()

        self.v_pos_encoder_thread.daemon = True
        self.v_pos_encoder_thread.start()

        self.v_zoom_encoder_thread.daemon = True
        self.v_zoom_encoder_thread.start()

    def data_copy_callback(self):
        while True:
            while self.plot_pipe_p.poll(0.001):

                data = self.plot_pipe_p.recv()
                if data == 'Start':
                    self.data_index = 0
                    self.x = np.full(1000000, np.nan)
                    self.z = np.full(1000000, np.nan)
                elif data == 'Stop':
                    pass
                    # Stop appending until told further
                else:
                    self.x[self.data_index] = data[0]
                    self.z[self.data_index] = data[1]
                    self.data_index += 1

    def _update_canvas(self, frame):
        # print(self.x)
        # print(self.z)

        # Check if x limits weren't manually updated
        if not self.x_lim_manual:
            self.x_lim = self._dynamic_ax.get_xlim()

        # Check if y limits weren't manually updated
        if not self.y_lim_manual:
            self.y_lim = self._dynamic_ax.get_ylim()

        # Clear the axes -- is there a better way to do this without resetting limits etc?
        # self._dynamic_ax.clear()

        # Reset the limits
        self._dynamic_ax.set_xlim(self.x_lim)
        self._dynamic_ax.set_ylim(self.y_lim)

        # If x lim was manually updated, push to nav stack
        if self.x_lim_manual:
            self.nav_toolbar.push_current()
            self._dynamic_ax.figure.canvas.draw()
            self.x_lim_manual = False

        # If y lim was manually updated, push to nav stack
        if self.y_lim_manual:
            self.nav_toolbar.push_current()
            self._dynamic_ax.figure.canvas.draw()
            self.y_lim_manual = False

        if not self.stop:
            self.y = np.sin(self.t + time.time())

        self.ln.set_data(self.x[0:self.data_index], self.z[0:self.data_index])

        if self.stop:
            self.ani.event_source.stop()

        return self.ln,

    def _draw_early_voltage(self):
        pass

    # ---------------BUTTON ON CLICK FUNCTIONS---------------

    def button_a_clicked(self):
        self.ani.event_source.stop()
        self.stop = True
        # self.button_a.setText("Test")

    def button_b_clicked(self):
        self.ani.event_source.start()
        self.stop = False

    def button_c_clicked(self):
        # Check if nav stack is empty, set this view as home if so
        self._nav_stack_empty()
        self.y_lim = [-20, 20]
        self.y_lim_manual = True
        self.update_once()

    def button_d_clicked(self):
        # Check if nav stack is empty, set this view as home if so
        self._nav_stack_empty()
        self.x_lim = [-20, 20]
        self.x_lim_manual = True
        self.update_once()

    def button_e_clicked(self):
        self.close()

    # ----- EXTERNAL BUTTON INPUTS -----

    def update_once(self, event=None):
        if self.stop:
            self.ani.event_source.start()

    def _nav_stack_empty(self):
        if self.nav_toolbar._nav_stack() is None:
            self.nav_toolbar.push_current()

    def horizontal_zoom_out(self, _):
        self._nav_stack_empty()
        self.x_lim = [2*i for i in self.x_lim]
        self.x_lim_manual = True
        self.update_once()

    def horizontal_zoom_in(self, _):
        self._nav_stack_empty()
        self.x_lim = [0.5*i for i in self.x_lim]
        self.x_lim_manual = True
        self.update_once()

    def vertical_zoom_out(self, _):
        self._nav_stack_empty()
        self.y_lim = [2*i for i in self.y_lim]
        self.y_lim_manual = True
        self.update_once()

    def vertical_zoom_in(self, _):
        self._nav_stack_empty()
        self.y_lim = [0.5*i for i in self.y_lim]
        self.y_lim_manual = True
        self.update_once()

    def pan_right(self, _):
        self._nav_stack_empty()
        interval = 0.05 * abs(self.x_lim[0] - self.x_lim[1])
        print(interval)
        self.x_lim = [i + interval for i in self.x_lim]
        print(self.x_lim)
        self.x_lim_manual = True
        self.update_once()

    def pan_left(self, _):
        self._nav_stack_empty()
        interval = 0.05 * abs(self.x_lim[0] - self.x_lim[1])
        self.x_lim = [i - interval for i in self.x_lim]
        self.x_lim_manual = True
        self.update_once()

    def pan_up(self, _):
        self._nav_stack_empty()
        interval = 0.05 * abs(abs(self.y_lim[0]) - abs(self.y_lim[1]))
        self.y_lim = [i + interval for i in self.y_lim]
        self.y_lim_manual = True
        self.update_once()

    def pan_down(self, _):
        self._nav_stack_empty()
        interval = 0.05 * abs(abs(self.y_lim[0]) - abs(self.y_lim[1]))
        self.y_lim = [i - interval for i in self.y_lim]
        self.y_lim_manual = True
        self.update_once()

    # ------- CANVAS ON CLICK EVENTS ---------

    def onclick(self, event):
        if self.nav_toolbar._active is None:
            print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
                  ('double' if event.dblclick else 'single', event.button,
                   event.x, event.y, event.xdata, event.ydata))

    def mouse(self, event):
        if self.nav_toolbar._active is 'ZOOM' or 'PAN':
            self.update_once()

    # ------- ENCODERS---------
    def my_callback(self, scale_position):
        print('Hello world! The scale position is {}'.format(scale_position))


class TeensyReceiver:
    # An instance of this should be instantiated by the main plotting UI
    # Plotting UI creates a pipe -> fed into instantiation
    # Teensy receiver async sends data through pipe for plotter to plot
    # Data type should be NP arrays
    # Pre-process raw data in receiving process
    # Plotting UI has callback every 50 ms to pull data into main arrays?
    # TODO: Need special characters for start/end of sequence

    # Adv over shared memory: Can clear at will, not fixed size

    import serial

    def __init__(self, teensy_pipe):
        self.teensy_pipe = teensy_pipe
        pass

    def __call__(self, *args, **kwargs):
        while True:
            self.teensy_pipe.send('Start')
            for i in range(10000):
                time.sleep(0.00001)
                self.teensy_pipe.send([i, i])


if __name__ == "__main__":
    qapp = QtWidgets.QApplication(sys.argv)
    app = ApplicationWindow()
    if not win32:
        app.encoders_start()
    app.showFullScreen()
    # app.show()
    qapp.exec_()
    sys.exit()
