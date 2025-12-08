from funcs import *

print(f"Введите номер файла: ")
num = input()

data = np.fromfile(f'bin/rx/rx_{num}.pcm', dtype=np.int16)

samples = data[0::2] + 1j * data[1::2]

N = len(samples)

samples = samples/np.max(np.real(samples))

L = np.ones(10)

sample = np.convolve(samples, L, mode='same')

sample = trim_signal_by_energy(sample)

symbols, error, offset_list, offset = sym_sync(sample)

symbols = costas_loop(symbols)

print(f'Найденный сдвиг: {offset}')

demapper(symbols, 5)

# create_dplot(np.arange(len(samples)), np.real(samples), 'b', "Реальная часть", np.arange(len(samples)), np.imag(samples), 'r', "Мнимая часть", "Индекс, n", "Амплитуда, А", "Исходные семплы")
# create_dplot(np.arange(len(error_list)), error_list, 'r', "Error", np.arange(len(offset_list)), offset_list, 'b', "offset", "", "", "TED")
# create_dplot(np.arange(len(sample)), np.real(sample), 'b', "Реальная часть", np.arange(len(sample)), np.imag(sample), 'r', "Мнимая часть", "Индекс, n", "Амплитуда, А", "Семплы после свертки")
create_constellation(np.real(symbols), np.imag(symbols), 'Реальная часть', "Мнимая часть", "Созвездие QPSK")
# create_plot(np.arange(len(error)), error)
plt.show()