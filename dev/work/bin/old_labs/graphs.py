from funcs import *

def estimate_cfo(rx_symbols, Nt, symbol_rate):
    if len(rx_symbols) < 2 * Nt:
        raise ValueError("Недостаточно символов для оценки CFO")

    x1 = rx_symbols[:Nt]
    x2 = rx_symbols[Nt:2*Nt]

    alpha_hat = np.sum(x2 * np.conj(x1))

    phase = np.angle(alpha_hat)

    est_cfo = phase / (2 * np.pi * (1.0 / symbol_rate))

    return est_cfo


def correct_cfo(samples, est_cfo, symbol_rate, n0=0):
    Ts = 1.0 / symbol_rate
    n = np.arange(len(samples)) + n0
    correction = np.exp(-1j * 2 * np.pi * est_cfo * n * Ts)
    return samples * correction

# print("Введите номер файла: ")
# num = input()

symbol_rate = 100_000      # 100 ksym/s
Fs = 1_000_000             # 1 Msps
Nsps = 10   # = 10
Nt = 13                    # длина Barker

print(f"Введите номер файла: ")
num = input()

if (num == "0"):
    data = np.fromfile('rx.pcm', dtype=np.int16)
else:
    data = np.fromfile(f'bin/rx/rx_{num}.pcm', dtype=np.int16)


samples = data[0::2] + 1j * data[1::2]
samples = samples / np.max(np.abs(samples))

coarse_symbols = samples[::Nsps]

est_cfo = estimate_cfo(coarse_symbols, Nt, symbol_rate)
print(f"Оценённый CFO: {est_cfo:.2f} Гц")

n = np.arange(len(samples))
correction = np.exp(-1j * 2 * np.pi * est_cfo * n / Fs)
corrected_samples = samples * correction

L = np.ones(Nsps)
filtered = np.convolve(corrected_samples, L, mode='same')

filtered = trim_signal_by_energy(filtered, Nsps=Nsps)

symbols, error, offset_list, offset = sym_sync(filtered)

symbols = costas_loop(symbols)

print(f"Найденный сдвиг: {offset}")

bits = demapper(symbols, 5)

print(bits)

create_dplot(np.arange(len(samples)), np.real(samples), 'b', "Re (до CFO)",
             np.arange(len(samples)), np.imag(samples), 'r', "Im (до CFO)",
             "Индекс, n", "Амплитуда", "Исходные отсчёты")

create_dplot(np.arange(len(filtered)), np.real(filtered), 'b', "Re (после CFO + MF)",
             np.arange(len(filtered)), np.imag(filtered), 'r', "Im (после CFO + MF)",
             "Индекс, n", "Амплитуда", "Отсчёты после коррекции")

create_constellation(np.real(symbols), np.imag(symbols),
                     "Re", "Im", f"Созвездие QPSK (CFO = {est_cfo:.1f} Гц)")

plt.show()

