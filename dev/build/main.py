from funcs import *

print(f"Введите номер файла: ")
num = input()

symbol_rate = 100000 
Fs = 1000000
Nsps = 10
Nt = 13
L = np.ones(Nsps)

if (num == "0"):
    data = np.fromfile('rx.pcm', dtype=np.int16)
else:
    data = np.fromfile(f'bin/rx/rx_{num}.pcm', dtype=np.int16)

signal = data[0::2] + 1j * data[1::2]
signal = signal / np.max(np.abs(signal))
# signal = signal[100000:500000]

filtered = np.convolve(signal, L, mode='same')
# filtered = trim_signal_by_energy(filtered, Nsps=Nsps)
# symbols, error, offset_list, offset = sym_sync(filtered)
symbols = filtered[::10]

f_cfo, autocor = cfo_estimation(symbols, symbol_rate, Nt)
print(f"Оценённый CFO: {f_cfo} Гц")

corrected_signal = cfo_correct(signal, f_cfo, Fs)
filtered_cfo = np.convolve(corrected_signal, L, mode='same')
filtered_cfo = costas_loop(filtered_cfo)
symbols_cfo, error, offset_list, offset = sym_sync(filtered_cfo)

# create_plot(np.arange(len(autocor)), autocor)
create_dplot(np.arange(len(signal)), np.real(signal), 'b', "Реальная часть", np.arange(len(signal)), np.imag(signal), 'r', "Мнимая часть", "Индекс, n", "Амплитуда, А", "Исходные семплы")
# create_dplot(np.arange(len(corrected_signal)), np.real(corrected_signal), 'b', "Реальная часть", np.arange(len(corrected_signal)), np.imag(corrected_signal), 'r', "Мнимая часть", "Индекс, n", "Амплитуда, А", "Correct семплы")
# create_dplot(np.arange(len(filtered)), np.real(filtered), 'b', "Реальная часть", np.arange(len(filtered)), np.imag(filtered), 'r', "Мнимая часть", "Индекс, n", "Амплитуда, А", "Семплы после свертки")
# create_dplot(np.arange(len(symbols)), np.real(symbols), 'b', "Реальная часть", np.arange(len(symbols)), np.imag(symbols), 'r', "Мнимая часть", "Индекс, n", "Амплитуда, А", "Исправленный CFO семплы")

create_constellation(np.real(filtered), np.imag(filtered), 'Реальная часть', "Мнимая часть", "Созвездие Costas Loop")
create_constellation(np.real(symbols), np.imag(symbols), 'Реальная часть', "Мнимая часть", "Созвездие Gardner")
create_constellation(np.real(symbols_cfo), np.imag(symbols_cfo), 'Реальная часть', "Мнимая часть", "Созвездие CFO")

plt.show()