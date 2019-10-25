import sys
import os
import time
from datetime import datetime

import numpy as np
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import QFileDialog

from matplotlib.backends.backend_qt5agg import (FigureCanvasQTAgg as FigureCanvas,
                                                NavigationToolbar2QT as NavigationToolbar)
from matplotlib.backend_bases import Event
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation

import multiprocessing as mp
import ctypes as c
import threading
import serial

if sys.platform != 'win32':
    from pyky040 import Encoder
    win32 = False
else:
    win32 = True

ARRAYSIZE = 1000000
# Set fixed size array for speed purposes. 1 million points is quite a lot (currently writing about 100k).


class ExpandingHeightButton(QtWidgets.QPushButton):
    def __init__(self, *argv):
        super().__init__(*argv)
        # Overload init to force fixed size
        self.setMaximumWidth(100)
        self.setSizePolicy(QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Expanding))


class ScrollMessageBox(QtWidgets.QMessageBox):
    # Create a scrollable message box
    def __init__(self, l, title, *args, **kwargs):
        QtWidgets.QMessageBox.__init__(self, *args, **kwargs)
        self.setWindowTitle(title)
        scroll = QtWidgets.QScrollArea(self)
        scroll.setWidgetResizable(True)
        self.content = QtWidgets.QWidget()
        scroll.setWidget(self.content)
        lay = QtWidgets.QVBoxLayout(self.content)
        for item in l:
            lay.addWidget(QtWidgets.QLabel(item, self))
        self.layout().addWidget(scroll, 0, 0, 1, self.layout().columnCount())
        self.setStyleSheet("QScrollArea{min-width:600 px; min-height: 300px}")


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

    # Overload save button for directories and filenames
    def save_figure(self, *args):
        directory = "/home/pi/Desktop/Curves/"
        filetypes = self.canvas.get_supported_filetypes_grouped()
        sorted_filetypes = sorted(filetypes.items())
        default_filetype = self.canvas.get_default_filetype()

        startpath = os.path.expanduser(
            directory)
        dt = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        start = os.path.join(startpath, dt + '.' + default_filetype)
        filters = []
        selectedFilter = None
        for name, exts in sorted_filetypes:
            exts_list = " ".join(['*.%s' % ext for ext in exts])
            filter = '%s (%s)' % (name, exts_list)
            if default_filetype in exts:
                selectedFilter = filter
            filters.append(filter)
        filters = ';;'.join(filters)

        fname, filter = QFileDialog.getSaveFileName(self.canvas.parent(),
                                         "Choose a filename to save to",
                                         start, filters, selectedFilter)
        if fname:
            # Save dir for next time, unless empty str (i.e., use cwd).
            if startpath != "":
                directory = (
                    os.path.dirname(fname))
            try:
                self.canvas.figure.savefig(fname)
            except Exception as e:
                QtWidgets.QMessageBox.critical(
                    self, "Error saving file", str(e),
                    QtWidgets.QMessageBox.Ok, QtWidgets.QMessageBox.NoButton)


class ApplicationWindow(QtWidgets.QMainWindow):
    # Currently, the home/back/forward + pan/zoom methods are laggy because they are running the old clear/draw
    # cycle in the background due to tight integration. External button inputs run much smoother as they only rely
    # on newer/faster drawing methods. Removing unnecessary draw calls would significantly speed up the process.

    def __init__(self):
        super().__init__()
        # stop/run variable
        self.stop = False

        # x limits
        self.x_lim = [-24, 24]
        self.x_lim_manual = True

        # y limits
        self.y_lim = [-0.2, 0.2]
        self.y_lim_manual = True

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

        # Plotted data arrays
        self.x_mp = mp.Array(c.c_double, ARRAYSIZE)
        self.x_np = np.frombuffer(self.x_mp.get_obj())  # This is to convert C shared memory array to NumPy array.
        # This creates an NP array with the memory pointing at the shared memory array
        # and the two share the same memory space.
        self.y_mp = mp.Array(c.c_double, ARRAYSIZE)
        self.y_np = np.frombuffer(self.y_mp.get_obj())
        self.downsample = 8

        # Full data arrays
        self.x_mp_full = mp.Array(c.c_double, ARRAYSIZE)
        self.x_np_full = np.frombuffer(self.x_mp_full.get_obj())
        self.y_mp_full = mp.Array(c.c_double, ARRAYSIZE)
        self.y_np_full = np.frombuffer(self.y_mp_full.get_obj())

        self.idx = mp.Value('i', 0)  # Index of where data is currently up to in shared memory
        self.mode = mp.Value('i', 0)  # 1 indicates Ic/Vbe, 2 indicates Ic/Vce
        self.overheat = mp.Value('i', 0)  # 1 indicates overheated, 2 is cooled down, 0 is not yet overheated
        self.is_npn = mp.Value('i', 1)  # NPN = 1, PNP = 0
        self.new_plot = mp.Value('i', 0)  # 1 indicates new plot started

        # Arrays for data
        self.sp_mp = mp.Array(c.c_double, ARRAYSIZE)  # Shunt+
        self.sn_mp = mp.Array(c.c_double, ARRAYSIZE)  # Shunt-
        self.vb_mp = mp.Array(c.c_double, ARRAYSIZE)  # Base voltage
        self.vc_mp = mp.Array(c.c_double, ARRAYSIZE)  # Collector voltage
        self.ve_mp = mp.Array(c.c_double, ARRAYSIZE)  # Emitter voltage
        self.vbr_mp = mp.Array(c.c_double, ARRAYSIZE)  # DAC value for voltage at base resistor (opposite side of resistor from DUT base)
        self.vbra_mp = mp.Array(c.c_double, ARRAYSIZE)  # Actual voltage for at base resistor

        self.sp = np.frombuffer(self.sp_mp.get_obj())
        self.sn = np.frombuffer(self.sn_mp.get_obj())
        self.vb = np.frombuffer(self.vb_mp.get_obj())
        self.vc = np.frombuffer(self.vc_mp.get_obj())
        self.ve = np.frombuffer(self.ve_mp.get_obj())
        self.vbr = np.frombuffer(self.vbr_mp.get_obj())
        self.vbra = np.frombuffer(self.vbra_mp.get_obj())

        # Early voltage curve data
        self.ep_index = 0
        self.ep_x = np.zeros(ARRAYSIZE//self.downsample)
        self.ep_y = np.zeros(ARRAYSIZE//self.downsample)
        self.ep_points = 30

        # Selected pointed/transconductance point data
        self.gm_index = 0
        self.gm_x = np.zeros(1)
        self.gm_y = np.zeros(1)

        # Set up subplots and axes -- including update timer
        self._dynamic_ax = dynamic_canvas.figure.subplots()
        # self._timer = dynamic_canvas.new_timer(
        #     400, [(self._update_canvas, (), {})])
        # self._timer.start()
        self._dynamic_ax.autoscale_view(False, False, False)
        self.ln = self._dynamic_ax.plot([], [], 'g.', scalex=False, scaley=False)[0]
        self.ep = self._dynamic_ax.plot([], [], 'b.', scalex=False, scaley=False)[0]
        self.gm_dot = self._dynamic_ax.plot([], [], 'm.', scalex=False, scaley=False)[0]


        # Create buttons
        buttons_layout = QtWidgets.QVBoxLayout()

        self.button_a = ExpandingHeightButton('Autoset')  # Autoset
        self.button_b = ExpandingHeightButton('Early Voltage')  # Early
        self.button_c = ExpandingHeightButton('Max Ic\nAlpha/Beta')  # Alpha/Beta
        self.button_d = ExpandingHeightButton("All\nAlpha/Beta")
        self.button_e = ExpandingHeightButton('Load')  # Exit
        self.button_f = ExpandingHeightButton('Exit')  # Exit and shutdown

        self.info_label = QtWidgets.QLabel("\n\n")
        self.info_label.setFixedWidth(100)

        # Add buttons to layout
        buttons_layout.addWidget(self.button_a)
        buttons_layout.addWidget(self.button_b)
        buttons_layout.addWidget(self.button_c)
        buttons_layout.addWidget(self.button_d)
        buttons_layout.addWidget(self.button_e)
        buttons_layout.addWidget(self.button_f)
        buttons_layout.addWidget(self.info_label)

        # Connect buttons to functions
        self.button_a.clicked.connect(self.button_a_clicked)
        self.button_b.clicked.connect(self.button_b_clicked)
        self.button_c.clicked.connect(self.button_c_clicked)
        self.button_d.clicked.connect(self.button_d_clicked)
        self.button_e.clicked.connect(self.button_e_clicked)
        self.button_f.clicked.connect(self.button_f_clicked)

        layout.addLayout(buttons_layout)
        self.info_label.setText("Welcome!\nPress Run\nor Single")

        # Events
        self.cid = self._dynamic_ax.figure.canvas.mpl_connect('button_press_event', self.onclick)
        self.move_mouse = self._dynamic_ax.figure.canvas.mpl_connect('motion_notify_event', self.mouse)
        # self.button_press = self._dynamic_ax.figure.canvas.mpl_connect('button_press_event', self.mouse)
        self.button_release = self._dynamic_ax.figure.canvas.mpl_connect('button_release_event', self.mouse)
        self.history = self._dynamic_ax.figure.canvas.mpl_connect('history', self.update_once)

        # Animate the canvas
        self.ani = FuncAnimation(self._dynamic_ax.figure, self._update_canvas, blit=True, interval=50)
        self._dynamic_ax.xaxis.label.set_color('red')
        self._dynamic_ax.yaxis.label.set_color('red')

        if not win32:
            # Initialise encoder threads
            self.h_pos_encoder = Encoder(CLK=38, DT=37, polling_interval=0.01)
            self.h_pos_encoder.setup(scale_min=-1e8, scale_max=1e8, step=1, inc_callback=self.pan_right,
                                     dec_callback=self.pan_left)
            self.h_pos_encoder_thread = threading.Thread(target=self.h_pos_encoder.watch)

            self.h_zoom_encoder = Encoder(CLK=16, DT=15, polling_interval=0.01)
            self.h_zoom_encoder.setup(scale_min=-1e8, scale_max=1e8, step=1, inc_callback=self.horizontal_zoom_in,
                                      dec_callback=self.horizontal_zoom_out)
            self.h_zoom_encoder_thread = threading.Thread(target=self.h_zoom_encoder.watch)

            self.v_pos_encoder = Encoder(CLK=33, DT=29, polling_interval=0.01)
            self.v_pos_encoder.setup(scale_min=-1e8, scale_max=1e8, step=1, inc_callback=self.pan_up,
                                     dec_callback=self.pan_down)
            self.v_pos_encoder_thread = threading.Thread(target=self.v_pos_encoder.watch)

            self.v_zoom_encoder = Encoder(CLK=31, DT=32, polling_interval=0.01)
            self.v_zoom_encoder.setup(scale_min=-1e8, scale_max=1e8, step=1, inc_callback=self.vertical_zoom_in,
                                      dec_callback=self.vertical_zoom_out)
            self.v_zoom_encoder_thread = threading.Thread(target=self.v_zoom_encoder.watch)


        # Actual Teensy receiver side
        self.teensy = TeensyReceiverSM(self.x_mp, self.y_mp, self.x_mp_full, self.y_mp_full, self.idx, self.mode,
                                       self.overheat, self.is_npn, self.downsample, self.sp_mp, self.sn_mp, self.vb_mp,
                                       self.vc_mp, self.ve_mp, self.vbr_mp, self.vbra_mp, self.new_plot)
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

    def _update_canvas(self, frame):

        # If new plot, reset all the appropriate variables
        if self.new_plot.value == 1:
            self._dynamic_ax.set_ylabel("Ic")
            if self.mode.value == 1:
                self._dynamic_ax.set_xlabel("Vbe")
            if self.mode.value == 2:
                self._dynamic_ax.set_xlabel("Vce")
            self._dynamic_ax.figure.canvas.draw()
            self.new_plot.value = 0
            self.ep_index = 0
            self.gm_index = 0
            self.button_b.setText("Early Voltage")
            self.info_label.setText("\n\n")

        # Display for if device overheats
        if self.overheat.value == 1:
            self.info_label.setText("Overheated!\n\n")
        elif self.overheat.value == 2:
            self.info_label.setText("Cooled down!\n\n")
            self.overheat.value = 0

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

        # Plot the data up to the appropriate indices
        index = int(self.idx.value)
        self.ln.set_data(self.x_np[0:index//self.downsample], self.y_np[0:index//self.downsample])
        self.ep.set_data(self.ep_x[0:self.ep_index], self.ep_y[0:self.ep_index])
        self.gm_dot.set_data(self.gm_x[0:self.gm_index], self.gm_y[0:self.gm_index])

        if self.stop:
            self.ani.event_source.stop()

        return self.ln, self.ep, self.gm_dot

    # ---------------BUTTON ON CLICK FUNCTIONS---------------

    def button_a_clicked(self):
        self.auto_set()

    def button_b_clicked(self):
        self.early_button()

    def button_c_clicked(self):
        self.alpha_beta()

    def button_d_clicked(self):
        self.alpha_beta_all()

    def button_e_clicked(self):
        self.calculate_load()

    def button_f_clicked(self):
        self.exit_app()

    # ---------------BUTTON CALLABLE FUNCTIONS---------------

    def exit_app(self):
        quit_msg = "Are you sure you want to exit?"
        reply = QtWidgets.QMessageBox.question(self, 'Quit?',
                                               quit_msg, QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.No)

        if reply == QtWidgets.QMessageBox.Yes:
            self.close()
        else:
            pass

    def auto_set(self):
        # Check if nav stack is empty, set this view as home if so
        if self.idx.value != 0:
            index_d = int(self.idx.value)//self.downsample
            self._nav_stack_empty()

            # If Early voltage needs to be plotted too
            if self.ep_index > 0:
                self.y_lim = [self.y_np[0:index_d].min(), self.y_np[0:index_d].max()]
                self.x_lim = [min(self.ep_x[0:self.ep_index].min(),self.x_np[0:index_d].min()),
                              max(self.x_np[0:index_d].max(), self.ep_x[0:self.ep_index].max())]

            # Otherwise just look at main data
            else:
                self.y_lim = [self.y_np[0:index_d].min(), self.y_np[0:index_d].max()]
                self.x_lim = [self.x_np[0:index_d].min(), self.x_np[0:index_d].max()]
            self.y_lim_manual = True
            self.x_lim_manual = True
            self.update_once()

    def find_end_curve_index(self):
        # Finds the end points of each curve
        # We can accomplish this as we one curve corresponds to one step in base voltage/current
        # This actual voltage can fluctuate, but since it is digitally controlled, we can read the control value
        ends = []
        index = int(self.idx.value)
        while index > 0:
            while self.vbr[index] == self.vbr[index - 1]:
                index -= 1
                if index <= 0:
                    break
            if index <= 0:
                break
            index -= 1
            ends.append(index)

        return np.array(ends)

    def calculate_load(self):
        if self.mode.value != 2:
            error_msg = "Load can be calculated only in Ic vs Vce mode!"
            reply = QtWidgets.QMessageBox.critical(self, 'Error',
                                                   error_msg, QtWidgets.QMessageBox.Ok)
        elif self.idx.value == 0:
            error_msg = "Load needs data to be calculated"
            reply = QtWidgets.QMessageBox.critical(self, 'Error',
                                                   error_msg, QtWidgets.QMessageBox.Ok)
        else:
            index = int(self.idx.value)

            # Max current point
            if self.is_npn.value:
                point = self.y_np_full[0:index].argmax()
            else:
                point = self.y_np_full[0:index].argmin()

            Ic_1 = self.y_np_full[point]
            Vce_1 = self.x_np_full[point]

            # Max voltage point
            if self.is_npn.value:
                point = self.x_np_full[0:index].argmax()
            else:
                point = self.x_np_full[0:index].argmin()

            Ic_2 = self.y_np_full[point]
            Vce_2 = self.x_np_full[point]

            load = abs(Vce_1-Vce_2) / abs(Ic_1 - Ic_2)
            self.info_label.setText(
                "Load: " + str(round(load, 2)) + "\nOhms\n")

    def early_button(self):
        if self.mode.value != 2:
            error_msg = "Early Voltage can be calculated only in Ic vs Vce mode!"
            reply = QtWidgets.QMessageBox.critical(self, 'Error',
                                               error_msg, QtWidgets.QMessageBox.Ok)
        elif self.idx.value == 0:
            error_msg = "Early Voltage needs data to be calculated"
            reply = QtWidgets.QMessageBox.critical(self, 'Error',
                                               error_msg, QtWidgets.QMessageBox.Ok)
        elif self.ep_index > 0:
            self.ep_index = 0
            self.button_b.setText("Early Voltage")
        else:
            ends = self.find_end_curve_index()
            points = len(ends)

            for idx in ends:
                if self.is_npn.value:
                    if self.y_np_full[idx] < 0.0001:
                        points -= 1
                else:
                    if self.y_np_full[idx] > -0.0001:
                        points -= 1

            if points == 0:
                error_msg = "Not enough curves to calculate Early Voltage!"
                reply = QtWidgets.QMessageBox.critical(self, 'Error',
                                                       error_msg, QtWidgets.QMessageBox.Ok)
                return
            early = np.full(points, np.nan)

            # Start at the end
            index = int(self.idx.value)

            for i in range(points):
                # Minus a few for good measure
                index -= 5

                # Note start index
                start_index = index

                # Find next index with Vce < 0.3 (NPN) or Vce > -0.3
                if self.is_npn.value:
                    while self.x_np_full[index] > 0.3:
                        index -= 1
                else:
                    while self.x_np_full[index] < -0.3:
                        index -= 1

                # Fit the curve and solve the root
                try:
                    # Fit a linear line
                    p = np.polyfit(self.x_np_full[index:start_index], self.y_np_full[index:start_index], 1)

                    # Ensure the Early voltage curves goes the correct direction
                    if p[0] > 0:
                        # Find the Early voltage
                        early[i] = np.roots(p)

                        # Evaluate some points and plot them
                        plot_x = np.linspace(early[i], self.x_np_full[start_index], self.ep_points)
                        plot_y = np.polyval(p, plot_x)
                        self.ep_x[self.ep_index:self.ep_index + self.ep_points] = plot_x
                        self.ep_y[self.ep_index:self.ep_index + self.ep_points] = plot_y
                        self.ep_index += self.ep_points
                except TypeError:
                    pass

                # Find where end of previous sweep was
                while self.vbr[index] == self.vbr[index-1]:
                    index -= 1
                index -= 1

            ans = float(np.nanmean(early))
            self.auto_set()
            self.info_label.setText("Early Voltage:\n" + str(round(ans, 2)) + " V\n")
            self.button_b.setText("Hide Early\nVoltage Plots")

    def alpha_beta(self):
        if self.idx.value > 0:
            index = int(self.idx.value)

            # Find end points where Ic > 0 for NPN, < 0 for PNP
            if self.is_npn.value:
                point = self.y_np_full[0:index].argmax()
            else:
                point = self.y_np_full[0:index].argmin()

            Ic = self.y_np_full[point]
            Ib = (self.vbra[point] - self.vb[point])/1000  # Resistor is 1 kOhm

            beta = Ic/Ib
            alpha = beta/(beta + 1)
            self.info_label.setText("Ic: " + str(round(Ic, 5)) + " A\nAlpha: " + str(round(alpha, 3)) + "\nBeta: " + str(round(beta, 2)))
        else:
            error_msg = "Alpha/Beta needs data to be calculated"
            reply = QtWidgets.QMessageBox.critical(self, 'Error',
                                                   error_msg, QtWidgets.QMessageBox.Ok)

    def alpha_beta_all(self):
        if self.mode.value != 2:
            error_msg = "All Alpha/Beta can be calculated only in Ic vs Vce mode! Max Alpha/Beta is calculatable " \
                        "in Ic vs Vbe mode though."
            reply = QtWidgets.QMessageBox.critical(self, 'Error',
                                                   error_msg, QtWidgets.QMessageBox.Ok)
            return

        ends = self.find_end_curve_index()

        # Find end points where Ic > 0 for NPN, < 0 for PNP
        if self.is_npn.value:
            ends = ends[self.y_np_full[ends] > 0]
        else:
            ends = ends[self.y_np_full[ends] < 0]

        Ic = self.y_np_full[ends]
        Ib = (self.vbra[ends] - self.vb[ends])/1000  # Resistor is 1 kOhm


        beta = Ic / Ib
        alpha = beta / (beta + 1)

        ends = ends[beta > 0]
        # See report for why this is done. Not properly reading voltages at amplifier and getting wrong readings

        string_list = []

        for i in range(len(ends)):
            string = ''
            string += "Ic: " + str(round(Ic[i], 5))
            string += " A, Ib: " + str(round(Ib[i], 5))
            string += " A, Beta: " + str(round(beta[i], 5))
            string += ", Alpha: " + str(round(alpha[i], 5))
            string_list.append(string)

        window = ScrollMessageBox(string_list, "All Alpha/Beta", None)
        window.exec_()

    def find_nearest(self, x, y):
        # As the x scale is typically much larger than the y scale, we need to scale the distance to account for this.
        # The user may hit one point on the screen, but as the axes are different scale, a different point from expected
        # can be selected.
        xrange = abs(self.x_lim[0] - self.x_lim[1])
        yrange = abs(self.y_lim[0] - self.y_lim[1])
        ratio = xrange/yrange

        # Only find values in the plotted array
        idx = self.idx.value//self.downsample
        dist = np.sqrt(np.square(self.x_np[0:idx] - x) + np.square(ratio*(self.y_np[0:idx] - y)))
        index = dist.argmin()
        return index

    def transconductance(self, index):
        return self.x_np[index], self.y_np[index], abs(self.y_np[index]/0.02585)

    # ----- EXTERNAL BUTTON INPUTS -----

    def update_once(self, event=None):
        if self.stop:
            self.ani.event_source.start()

    def _nav_stack_empty(self):
        if self.nav_toolbar._nav_stack() is None:
            self.nav_toolbar.push_current()

    def horizontal_zoom_out(self, _):
        self._nav_stack_empty()
        self.x_lim = [1.4142135623730951*i for i in self.x_lim]  # sqrt(2)
        self.x_lim_manual = True
        self.update_once()

    def horizontal_zoom_in(self, _):
        self._nav_stack_empty()
        self.x_lim = [0.7071067811865475*i for i in self.x_lim]  # 1/sqrt(2)
        self.x_lim_manual = True
        self.update_once()

    def vertical_zoom_out(self, _):
        self._nav_stack_empty()
        self.y_lim = [1.4142135623730951*i for i in self.y_lim]  # sqrt(2)
        self.y_lim_manual = True
        self.update_once()

    def vertical_zoom_in(self, _):
        self._nav_stack_empty()
        self.y_lim = [0.7071067811865475*i for i in self.y_lim]  # 1/sqrt(2)
        self.y_lim_manual = True
        self.update_once()

    def pan_right(self, _):
        self._nav_stack_empty()
        interval = 0.05 * abs(self.x_lim[0] - self.x_lim[1])
        self.x_lim = [i + interval for i in self.x_lim]
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
        if self.nav_toolbar._active is None and self.idx.value > 0:
            print(event.xdata, event.ydata)
            index = self.find_nearest(event.xdata, event.ydata)
            Vce, Ic, gm = self.transconductance(index)
            self.info_label.setText("Vce: " + str(round(Vce, 2)) + " V\nIc: " + str(round(Ic, 5)) + " A\ngm: " +
                                    str(round(gm, 2)))
            self.gm_x[0] = Vce
            self.gm_y[0] = Ic
            self.gm_index = 1
            # print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
            #       ('double' if event.dblclick else 'single', event.button,
            #        event.x, event.y, event.xdata, event.ydata))

    def mouse(self, event):
        # This forces an axes update when zooming/panning with matplotlib toolbar
        if self.nav_toolbar._active is 'ZOOM' or 'PAN':
            self.update_once()


class ReadLine:
    def __init__(self, ser):
        self.buf = bytearray()
        self.s = ser  # Serial object
        self.i = 0  # Index for buffer to indicate how much data left

    def readline(self):
        # Find newline in buffer
        self.i = self.buf.find(b"\n")

        # If points lines still in buffer, read them out first
        if self.i > 0:
            r = self.buf[:self.i+1]
            self.buf = self.buf[self.i+1:]
            return r

        # If the buffer is empty, read in as many lines as available
        while True:
            self.i = max(0, min(2048, self.s.in_waiting))
            data = self.s.read(self.i)
            self.i = data.find(b"\n")
            if self.i > 0:
                r = self.buf + data[:self.i+1]
                self.buf[0:] = data[self.i+1:]
                return r
            else:
                self.buf.extend(data)


class TeensyReceiverSM:
    def __init__(self, x_array, y_array, x_full, y_full, idx, mode, overheat, is_npn, downsample, sp_mp, sn_mp, vb_mp,
                 vc_mp, ve_mp, vbr_mp, vbra_mp, new_plot):
        
        # Arrays for plotted data
        self.x_mp = x_array
        self.y_mp = y_array
        
        # Arrays for full sets of data
        self.x_full = x_full
        self.y_full = y_full
        self.x_full_np = np.frombuffer(self.x_full.get_obj())
        self.y_full_np = np.frombuffer(self.y_full.get_obj())
        
        self.idx = idx  # Index in array when data is currently up to
        self.mode = mode  # 1 = Ic/Vbe, 2 = Ic/Vce
        self.overheat = overheat  # Is overheated?
        self.is_npn = is_npn  # Is NPN? 1 is NPN
        self.downsample = downsample  # Downsampling factor
        self.new_plot = new_plot  # Indicates whether new plot is started

        # Arrays for all data
        self.sp_mp = sp_mp
        self.sn_mp = sn_mp
        self.vb_mp = vb_mp
        self.vc_mp = vc_mp
        self.ve_mp = ve_mp
        self.vbr_mp = vbr_mp
        self.vbra_mp = vbra_mp

        self.sp = np.frombuffer(self.sp_mp.get_obj())
        self.sn = np.frombuffer(self.sn_mp.get_obj())
        self.vb = np.frombuffer(self.vb_mp.get_obj())
        self.vc = np.frombuffer(self.vc_mp.get_obj())
        self.ve = np.frombuffer(self.ve_mp.get_obj())
        self.vbr = np.frombuffer(self.vbr_mp.get_obj())
        self.vbra = np.frombuffer(self.vbra_mp.get_obj())

        # Conversion factors
        self.factor = 3.3*8.2/8192  # ADC with voltage divider
        self.dac_factor = 3.3*9.18/4096  # DAC with amp gain

    def __call__(self, *args, **kwargs):
        ser = serial.Serial('/dev/ttyACM0', 2000000)
        rl = ReadLine(ser)
        sweep_ongoing = False
        while True:
            if ser.in_waiting > 0 or rl.i > 0:
                # Read the line and convert to string
                line = rl.readline()
                line = line.decode()
                line = line.strip()

                # Start of new sweep
                if line == "++":
                    print("Start")
                    sweep_ongoing = True
                    self.idx.value = 0
                    self.mode.value = 0
                    dt = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                    f = open("/home/pi/Desktop/Curves/" + dt + "_raw.txt", "w")
                    f_points = open("/home/pi/Desktop/Curves/" + dt + "_points.txt", "w")
                    continue

                # End of sweep
                if line == "--":
                    print("End")
                    sweep_ongoing = False
                    f.close()
                    f_points.close()
                    continue

                # Overheated
                if line == "OV":
                    self.overheat.value = 1
                    sweep_ongoing = False
                    print("OV")
                    continue
                # Cooled down
                elif line == "CD":
                    self.overheat.value = 2
                    print("CD")
                    continue

                if line == "NPN":
                    self.is_npn.value = 1
                    print("NPN")
                    continue
                elif line == "PNP":
                    self.is_npn.value = 0
                    print("PNP")
                    continue

                # Once sweep started, listen for what mode
                if sweep_ongoing and self.mode.value == 0:
                    if line == "M1":
                        print("M1")
                        #  Ic vs Vbe
                        self.mode.value = 1
                        self.new_plot.value = 1
                        continue
                    elif line == "M2":
                        print('M2')
                        # Ic vs Vce
                        self.mode.value = 2
                        self.new_plot.value = 1
                        continue

                # Write the raw data to file
                f.write(line + "\n")

                if sweep_ongoing and (self.mode.value != 0):  # If mode is 0, sweep is not active
                    string = line.split(',')
                    if len(string) != 7:
                        continue
                        # If we don't have 7 data points, something has gone wrong. Transmission/reading error.

                    index = int(self.idx.value)

                    # Put data into arrays
                    self.sp_mp[index] = float(string[0]) * self.factor
                    self.sn_mp[index] = float(string[1]) * self.factor
                    self.vb_mp[index] = float(string[2]) * self.factor
                    self.vc_mp[index] = float(string[3]) * self.factor
                    self.ve_mp[index] = float(string[4]) * self.factor
                    self.vbr_mp[index] = float(string[5]) * self.dac_factor
                    self.vbra_mp[index] = float(string[6]) * self.factor

                    # Calculate the x/y points
                    if self.mode.value == 1:  # Ic vs Vbe
                        self.x_full[index] = self.vb[index] - self.ve[index]
                        self.y_full[index] = (self.sp[index] - self.sn[index])/10

                        if index % self.downsample == 0:
                            self.x_mp[index // self.downsample] = self.x_full[index]
                            self.y_mp[index // self.downsample] = self.y_full[index]

                    elif self.mode.value == 2:  # Ic vs Vce
                        self.x_full[index] = self.vc[index] - self.ve[index]
                        self.y_full[index] = (self.sp[index] - self.sn[index]) / 10

                        if index % self.downsample == 0:
                            self.x_mp[index // self.downsample] = self.x_full[index]
                            self.y_mp[index // self.downsample] = self.y_full[index]

                    # Write the points to file
                    f_points.write(str(self.x_full[index]) + "," + str(self.y_full[index]))

                    # Increment the index
                    self.idx.value += 1


if __name__ == "__main__":

    qapp = QtWidgets.QApplication(sys.argv)
    app = ApplicationWindow()
    if not win32:
        app.encoders_start()
    app.showFullScreen()
    qapp.exec_()
    sys.exit()


