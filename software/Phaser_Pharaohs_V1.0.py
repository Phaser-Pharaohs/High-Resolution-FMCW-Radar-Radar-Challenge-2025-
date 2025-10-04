# === Imports ===
import sys
import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout, QLabel, QPushButton, QSlider
from pyqtgraph.Qt import QtCore, QtGui
import adi
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
plt.ion()


# === Parameters ===
c = 3e8
sample_rate = 5e6
center_freq = 1.5e9
signal_freq = 100e3
rx_gain = 0  # dB # must be between -3 and 70
tx_gain = -20   # must be between 0 and -88
output_freq = 8e9
default_chirp_bw = 3000e6
ramp_time = 300  # microseconds
No_steps = 300
Range_lim = 10  # m
auto_steer_angle = 0
auto_steer_step = 2  # degrees
auto_steer_min = -45
auto_steer_max = 45
num_chirps = 2


# === Connect to hardware ===
rpi_ip = "ip:phaser.local"
sdr_ip = "ip:192.168.2.1"
my_sdr = adi.ad9361(uri=sdr_ip)
my_phaser = adi.CN0566(uri=rpi_ip, sdr=my_sdr)


# === Phaser setup ===
my_phaser.configure(device_mode="rx")
my_phaser.element_spacing = 0.014
my_phaser.load_gain_cal()
my_phaser.load_phase_cal()

for i in range(8):
    my_phaser.set_chan_phase(i, 0)

gain_list = [8, 34, 84, 127, 127, 84, 34, 8]
# gain_list = [127] * 8
for i, g in enumerate(gain_list):
    my_phaser.set_chan_gain(i, g, apply_cal=True)


# GPIO setup
my_phaser._gpios.gpio_tx_sw = 0  # 0 = TX_OUT_2, 1 = TX_OUT_1
my_phaser._gpios.gpio_vctrl_1 = 1 # 1=Use onboard PLL/LO source  (0=disable PLL and VCO, and set switch to use external LO input)
my_phaser._gpios.gpio_vctrl_2 = 1 # 1=Send LO to transmit circuitry  (0=disable Tx path, and send LO to LO_OUT)

# === SDR Rx ===
my_sdr.sample_rate = int(sample_rate)
sample_rate = int(my_sdr.sample_rate)
my_sdr.rx_lo = int(center_freq)
my_sdr.rx_enabled_channels = [0, 1]
my_sdr.gain_control_mode_chan0 = "manual"
my_sdr.gain_control_mode_chan1 = "manual"
my_sdr.rx_hardwaregain_chan0 = int(rx_gain)
my_sdr.rx_hardwaregain_chan1 = int(rx_gain)

# === SDR Tx ===
my_sdr.tx_lo = int(center_freq)
my_sdr.tx_enabled_channels = [0, 1]
my_sdr.tx_cyclic_buffer = True  # must set cyclic buffer to true for the tdd burst mode.
my_sdr.tx_hardwaregain_chan0 = -88  # must be between 0 and -88
my_sdr.tx_hardwaregain_chan1 = tx_gain  # must be between 0 and -88


# === PLL config ===
BW = default_chirp_bw
num_steps = int(No_steps)
my_phaser.frequency = int((output_freq + signal_freq + center_freq) / 4)
my_phaser.freq_dev_range = int(BW / 4)
my_phaser.freq_dev_step = int((BW / 4) / num_steps)
my_phaser.freq_dev_time = int(ramp_time)
my_phaser.delay_word = 4095
my_phaser.delay_clk = "PFD"
my_phaser.ramp_mode = "single_sawtooth_burst"  # ramp_mode can be:  "disabled", "continuous_sawtooth", "continuous_triangular", "single_sawtooth_burst", "single_ramp_burst"
my_phaser.tx_trig_en = 1 # start a ramp with TXdata
my_phaser.enable = 0    # 0 = PLL enable.  Write this last to update all the registers
my_phaser._gpios.gpio_burst = 0


# === Timing and FFT sizing ===
ramp_time = int(my_phaser.freq_dev_time)
ramp_time_s = ramp_time / 1e6
slope = BW / ramp_time_s
begin_offset_time = 0.2 * ramp_time_s
good_ramp_samples = int((ramp_time_s - begin_offset_time) * sample_rate)
start_offset_samples = int(begin_offset_time * sample_rate)


# %%
""" Synchronize chirps to the start of each Pluto receive buffer
"""
# Configure TDD controller
sdr_pins = adi.one_bit_adc_dac(sdr_ip)
sdr_pins.gpio_tdd_ext_sync = True # If set to True, this enables external capture triggering using the L24N GPIO on the Pluto.  When set to false, an internal trigger pulse will be generated every second
tdd = adi.tddn(sdr_ip)
sdr_pins.gpio_phaser_enable = True
tdd.enable = False         # disable TDD to configure the registers
tdd.sync_external = True
tdd.startup_delay_ms = 0
PRI_ms = ramp_time/1e3 + 0.2
tdd.frame_length_ms = PRI_ms    # each chirp is spaced this far apart
tdd.burst_count = num_chirps       # number of chirps in one continuous receive buffer

tdd.channel[0].enable = True
tdd.channel[0].polarity = False
tdd.channel[0].on_raw = 0
tdd.channel[0].off_raw = 10
tdd.channel[1].enable = True
tdd.channel[1].polarity = False
tdd.channel[1].on_raw = 0
tdd.channel[1].off_raw = 10
tdd.channel[2].enable = True
tdd.channel[2].polarity = False
tdd.channel[2].on_raw = 0
tdd.channel[2].off_raw = 10
tdd.enable = True

# size the fft for the number of ramp data points
power=8
fft_size = int(2**power)
num_samples_frame = int(tdd.frame_length_ms/1000*sample_rate)
while num_samples_frame > fft_size:
    power=power+1
    fft_size = int(2**power)
    if power==18:
        break
print("fft_size =", fft_size)

num_slices = int(((auto_steer_max - auto_steer_min) / auto_steer_step) + 1)

prev= np.zeros((num_slices,fft_size))
current= np.zeros((num_slices,fft_size))

# Pluto receive buffer size needs to be greater than total time for all chirps
total_time = tdd.frame_length_ms * num_chirps   # time in ms
print("Total Time for all Chirps:  ", total_time, "ms")
buffer_time = 0
power=8
while total_time > buffer_time:
    power=power+1
    buffer_size = int(2**power)
    buffer_time = buffer_size/my_sdr.sample_rate*1000   # buffer time in ms
    if power==23:
        break     # max pluto buffer size is 2**23, but for tdd burst mode, set to 2**22
print("buffer_size:", buffer_size)
my_sdr.rx_buffer_size = buffer_size
print("buffer_time:", buffer_time, " ms")


print(
    """
CONFIG:
Sample rate: {sample_rate}MHz
Num samples: 2^{Nlog2}
Bandwidth: {BW}MHz
Ramp time: {ramp_time}ms
Output frequency: {output_freq}MHz
IF: {signal_freq}kHz
""".format(
        sample_rate=sample_rate / 1e6,
        Nlog2=int(np.log2(my_sdr.rx_buffer_size)),
        BW=BW / 1e6,
        ramp_time=ramp_time / 1e3,
        output_freq=output_freq / 1e6,
        signal_freq=signal_freq / 1e3,
    )
)

# %%
""" Create a sinewave waveform for Pluto's transmitter
"""
N = int(2**18)
fc = int(signal_freq)
ts = 1 / float(sample_rate)
t = np.arange(0, N * ts, ts)
i = np.cos(2 * np.pi * t * fc) * 2 ** 14
q = np.sin(2 * np.pi * t * fc) * 2 ** 14
iq = 1 * (i + 1j * q)

# transmit data from Pluto
my_sdr._ctx.set_timeout(30000)
my_sdr._rx_init_channels()
my_sdr.tx([iq, iq])

# === GUI Window ===

class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PHASER FMCW Radar")
        self.setGeometry(0, 0, 400, 400)  # (x,y, width, height)
        # self.setFixedWidth(600)
        self.setWindowState(QtCore.Qt.WindowMaximized)
        self.num_rows = 12
        self.setWindowFlag(QtCore.Qt.WindowCloseButtonHint, False) #remove the window's close button
        self.UiComponents()
        self.show()

    # method for components
    def UiComponents(self):
        widget = QWidget()
        layout = QGridLayout()

        # Quit button
        self.quit_button = QPushButton("Quit")
        self.quit_button.pressed.connect(self.end_program)
        layout.addWidget(self.quit_button, 30, 0, 4, 4)

        # Control Panel
        control_label = QLabel("PHASER FMCW Radar")
        font = control_label.font()
        font.setPointSize(24)
        control_label.setFont(font)
        font.setPointSize(12)
        control_label.setAlignment(Qt.AlignHCenter)  # | Qt.AlignVCenter)
        layout.addWidget(control_label, 0, 0, 1, 2)


        # waterfall level slider
        self.low_slider = QSlider(Qt.Horizontal)
        self.low_slider.setMinimum(0)
        self.low_slider.setMaximum(100)
        self.low_slider.setValue(0)
        self.low_slider.setTickInterval(20)
        self.low_slider.setMaximumWidth(200)
        self.low_slider.setTickPosition(QSlider.TicksBelow)
        self.low_slider.valueChanged.connect(self.get_water_levels)
        layout.addWidget(self.low_slider, 8, 0)

        self.high_slider = QSlider(Qt.Horizontal)
        self.high_slider.setMinimum(0)
        self.high_slider.setMaximum(100)
        self.high_slider.setValue(10)
        self.high_slider.setTickInterval(20)
        self.high_slider.setMaximumWidth(200)
        self.high_slider.setTickPosition(QSlider.TicksBelow)
        self.high_slider.valueChanged.connect(self.get_water_levels)
        layout.addWidget(self.high_slider, 10, 0)

        self.water_label = QLabel("Plot Intensity Levels")
        self.water_label.setFont(font)
        self.water_label.setAlignment(Qt.AlignCenter)
        self.water_label.setMinimumWidth(100)
        self.water_label.setMaximumWidth(200)


        title_style = {"size": "20pt"}
        label_style = {"color": "#FFF", "font-size": "14pt"}

        # Waterfall plot
        self.waterfall = pg.PlotWidget()
        self.imageitem = pg.ImageItem()
        self.waterfall.addItem(self.imageitem)
        # Use a viridis colormap
        pos = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
        color = np.array([[68, 1, 84,255], [59, 82, 139,255], [33, 145, 140,255], [94, 201, 98,255], [253, 231, 37,255]], dtype=np.ubyte)
        lut = pg.ColorMap(pos, color).getLookupTable(0.0, 1.0, 256)
        self.imageitem.setLookupTable(lut)
        self.imageitem.setLevels([0,1])
        tr = QtGui.QTransform()
        m_per_bin = (sample_rate / fft_size) * c / (2 * slope)
        signal_freq_offset=signal_freq/slope*1e6*150
        #tr.translate(auto_steer_min, -m_per_bin *fft_size/2-signal_freq_offset)  # start at 0 m
        tr.translate(auto_steer_min, - signal_freq_offset)  # start at 0 m
        tr.scale(auto_steer_step, m_per_bin)

        self.imageitem.setTransform(tr)
        # zoom_freq = 35e3
        # self.waterfall.setRange(yRange=(signal_freq, signal_freq + zoom_freq))
        self.waterfall.setRange(yRange=(0, Range_lim))
        self.waterfall.setTitle("Range Azimuth Map", **title_style)
        self.waterfall.setLabel("left", "Range", units="m", **label_style)
        self.waterfall.setLabel("bottom", "Angle", units="Degree", **label_style)
        layout.addWidget(self.waterfall, 0 + self.num_rows + 1, 2, self.num_rows, 1)
        self.img_array = np.ones((num_slices, fft_size))*(-100)

        widget.setLayout(layout)
        # setting this widget as central widget of the main window
        self.setCentralWidget(widget)

    def get_water_levels(self):
        if self.low_slider.value() > self.high_slider.value():
            self.low_slider.setValue(self.high_slider.value())
        self.imageitem.setLevels([self.low_slider.value(), self.high_slider.value()])

    def end_program(self):
        my_sdr.tx_destroy_buffer()
        print("Program finished and Pluto Tx Buffer Cleared")
        # disable TDD and revert to non-TDD (standard) mode
        tdd.enable = False
        sdr_pins.gpio_phaser_enable = False
        tdd.channel[1].polarity = not(sdr_pins.gpio_phaser_enable)
        tdd.channel[2].polarity = sdr_pins.gpio_phaser_enable
        tdd.enable = True
        tdd.enable = False
        self.close()

def update():
    global auto_steer_angle, prev, current, rx_bursts

    my_phaser._gpios.gpio_burst = 1
    my_phaser._gpios.gpio_burst = 0

    # Beam steering
    phase_delta = (2 * np.pi * output_freq * my_phaser.element_spacing *
                   np.sin(np.radians(auto_steer_angle)) / c)
    my_phaser.set_beam_phase_diff(np.degrees(phase_delta))

    # Receive and process
    data = my_sdr.rx()
    sum_data = data[0] + data[1]

    # select just the linear portion of the last chirp
    rx_bursts = np.zeros((num_chirps, good_ramp_samples), dtype=complex)
    avg_burst_data = np.zeros(fft_size, dtype=complex)
    for burst in range(num_chirps):
        start_index = start_offset_samples + burst * num_samples_frame
        stop_index = start_index + good_ramp_samples
        rx_bursts[burst] = sum_data[start_index:stop_index]
        burst_data = np.ones(fft_size, dtype=complex) * 1e-10
        win_funct = np.blackman(len(rx_bursts[burst]))# win_funct = np.ones(len(rx_bursts[burst]))
        burst_data[start_offset_samples:(start_offset_samples + good_ramp_samples)] = rx_bursts[burst] * win_funct
        avg_burst_data += burst_data


    # plt.clf()
    # # plt.scatter(rx_bursts.real[0],rx_bursts.imag[0])
    # plt.plot(np.real(rx_bursts[0]))
    # plt.plot(np.real(rx_bursts[1]))
    # plt.pause(0.1)
    # sp = np.absolute(np.fft.fft(avg_burst_data))

    # Store in current frame for waterfall
    frame = int((auto_steer_angle + auto_steer_max) / auto_steer_step)
    current[frame] = sp/ np.sum(win_funct)#s_mag



    # Update steering angle and image
    auto_steer_angle += auto_steer_step
    if auto_steer_angle > auto_steer_max:
        auto_steer_angle = auto_steer_min
        # win.img_array = np.abs(current - prev)
        win.img_array = np.abs(current)
        prev = np.copy(current)
        # Ensure pyqtgraph has levels for float data
        win.imageitem.setLevels([win.low_slider.value(), win.high_slider.value()])
        win.imageitem.setImage(win.img_array, autoLevels=False)
def debug_plot(
    data,
    title="Debug Plot",
    ylabel="Amplitude",
    xlabel="Sample Index",
    use_dB=False,
    use_freq=False,
    fs=None
):
    """
    Quick debug plot utility for 1D signals like FFTs or chirps.

    Args:
        data (array-like): Signal to plot (1D real or complex).
        title (str): Plot title.
        ylabel (str): Y-axis label.
        xlabel (str): X-axis label.
        use_dB (bool): If True, plots 20*log10(magnitude).
        use_freq (bool): If True, x-axis is frequency (requires fs).
        fs (float): Sample rate in Hz (required if use_freq=True).
    """
    import numpy as np
    import matplotlib.pyplot as plt

    if use_dB:
        y = 20 * np.log10(np.maximum(np.abs(data), 1e-15))
        ylabel = "Magnitude (dB)"
    else:
        y = np.real(data) if np.iscomplexobj(data) else data

    if use_freq and fs is not None:
        x = np.linspace(-fs / 2, fs / 2, len(data))
        xlabel = "Frequency (Hz)"
    else:
        x = np.arange(len(data))

    plt.figure()
    plt.plot(x, y)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# === Main ===
App = QApplication(sys.argv)
win = Window()
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(0)
sys.exit(App.exec())


