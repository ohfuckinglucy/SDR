from funcs import *

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
trimmed = trim_signal_by_energy(signal, Nsps=Nsps, threshold_ratio=0.3)
filtered = np.convolve(signal, L, mode='same')

symbols, error, offset_list, offset = sym_sync(filtered)

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

# symbols = costas_loop(symbols_filtered) 

create_dplot(np.arange(len(signal)), np.real(signal), 'b', "Реальная часть", np.arange(len(signal)), np.imag(signal), 'r', "Мнимая часть", "Индекс, n", "Амплитуда, А", "Исходные семплы")
# # create_dplot(np.arange(len(symbols)), np.real(symbols), 'b', "Реальная часть", np.arange(len(symbols)), np.imag(symbols), 'r', "Мнимая часть", "Индекс, n", "Амплитуда, А", "Исправленный CFO семплы")

create_constellation(np.real(symbols), np.imag(symbols), 'Реальная часть', "Мнимая часть", "Созвездие Gardner")

plt.show()