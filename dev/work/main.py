from vispy import app, scene
app.use_app('pyqt6')

from funcs import *
from vispy import plot as vp

print(f"Введите номер файла: ")
num = input()

symbol_rate = 100000 
Fs = 1000000
Nsps = 10
Nt = 13
L = np.ones(Nsps)

if (num == "0"):
    data = np.fromfile('../build/rx.pcm', dtype=np.int16)
else:
    data = np.fromfile(f'bin/rx/rx_{num}.pcm', dtype=np.int16)

signal = data[0::2] + 1j * data[1::2]
signal = signal / np.max(np.abs(signal))

sync_seq = [1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1]
sync_sym = [(1 - 2*b) * (1 + 1j) / np.sqrt(2) for b in sync_seq]

sync_sym_ul = np.zeros(len(sync_sym) * Nsps, dtype=complex)
sync_sym_ul[::Nsps] = sync_sym
sync_sym_ul_filtered = np.convolve(sync_sym_ul, L, mode='full')

filtered = np.convolve(signal, L, mode='same')

corr = normalized_correlation(signal, sync_sym_ul_filtered)

start = np.argmax(np.abs(corr))

print(f"Начало сигнала найдено на отсчёте: {start}")

filtered = filtered[start:]
filtered = costas_loop(filtered)
symbols, error, offset_list, offset = sym_sync(filtered)

symbols = np.array(symbols)

# barker_symbols = symbols_trimmed[:26]
# autocor = cfo_corr(barker_symbols, Nt=13)
# corr = autocor[0]
# epsilon_hat = np.angle(corr) / (2 * np.pi * Nt)
# f_cfo = epsilon_hat * symbol_rate 
# print(f"Estimated CFO: {f_cfo:.2f} Hz")
# n = np.arange(len(signal))

# signal_corrected = signal * np.exp(-1j * 2 * np.pi * f_cfo * n / Fs)

# filtered_cor = np.convolve(signal_corrected, L, mode='same')
# symbols, error, offset_list, offset = sym_sync(filtered_cor)

canvas1 = scene.SceneCanvas(
    title="Временной сигнал (I/Q)",
    size=(1200, 400),
    bgcolor='#0d0d0d',
    show=True
)
grid1 = canvas1.central_widget.add_grid()
view1 = grid1.add_view(0, 0, bgcolor='#0d0d0d')

n_samples = len(symbols)
time_axis = np.arange(n_samples)

line_I = scene.Line(
    pos=np.column_stack([time_axis, filtered.real[:n_samples]]),
    color='#4DA6FF',
    width=2.0,
    parent=view1.scene
)

line_Q = scene.Line(
    pos=np.column_stack([time_axis, filtered.imag[:n_samples]]),
    color='#FF6666',
    width=2.0,
    parent=view1.scene
)

x_axis = scene.AxisWidget(orientation='bottom', text_color='white')
y_axis = scene.AxisWidget(orientation='left', text_color='white')
x_axis.stretch = (1, 0.1)
y_axis.stretch = (0.1, 1)

grid1.add_widget(x_axis, row=1, col=0)
grid1.add_widget(y_axis, row=0, col=1)
grid1.add_widget(view1, row=0, col=0)

x_axis.link_view(view1)
y_axis.link_view(view1)

view1.camera = scene.PanZoomCamera(rect=(0, -1.5, n_samples, 3))

canvas2 = scene.SceneCanvas(
    title="Созвездие QPSK/BPSK",
    size=(600, 600),
    bgcolor='#0d0d0d',
    show=True
)
view2 = canvas2.central_widget.add_view(bgcolor='#0d0d0d')

max_points = 10000
if len(symbols) > max_points:
    idx = np.random.choice(len(symbols), max_points, replace=False)
    symbols_plot = symbols[idx]
else:
    symbols_plot = symbols

scatter = scene.Markers(
    parent=view2.scene,
    pos=np.column_stack([symbols_plot.real, symbols_plot.imag]),
    face_color=(0.2, 1.0, 0.2, 0.6),
    edge_color=None,
    size=8
)

x_axis2 = scene.AxisWidget(orientation='bottom', text_color='white')
y_axis2 = scene.AxisWidget(orientation='left', text_color='white')
x_axis2.stretch = (1, 0.1)
y_axis2.stretch = (0.1, 1)

grid2 = canvas2.central_widget.add_grid()
grid2.add_widget(x_axis2, row=1, col=0)
grid2.add_widget(y_axis2, row=0, col=1)
grid2.add_widget(view2, row=0, col=0)

x_axis2.link_view(view2)
y_axis2.link_view(view2)

view2.camera = scene.PanZoomCamera(rect=(-2, -2, 4, 4))

app.run()