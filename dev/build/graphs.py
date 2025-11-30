import numpy as np
from matplotlib import pyplot as plt

## Открытие файла и формирование семплов
print("Tx: 0\nRx: 1")
choose = int(input())

if (choose == 0):
    data = np.fromfile("bin/lab6/tx_1.pcm", dtype=np.int16)
else:
    data = np.fromfile("bin/lab6/rx_2.pcm", dtype=np.int16)

N = len(data) // 2

samples_all = []
for i in range(N):
    samples_all.append(((data[2*i]) + 1j * (data[2*i+1]))/np.max(data))

samples = []
for i in range(len(samples_all)):
    if samples_all[i] >= 0.05:
        samples.append(samples_all[i])

L = np.ones(10)

sample = np.convolve(samples, L, mode='same')

symbols = []

for i in range(9, len(sample), 10):
    symbols.append(sample[i])

bits = []
for s in symbols:
    re = np.real(s)
    im = np.imag(s)
    if re >= 0 and im >= 0:
        bits.append(0)
        bits.append(0)
    elif re < 0 and im >= 0:
        bits.append(0)
        bits.append(1)
    elif re < 0 and im < 0:
        bits.append(1)
        bits.append(1)
    else:
        bits.append(1)
        bits.append(0)

print(f'Длина битной последовательности: {len(bits)}\nБиты: {bits}')

plt.figure(figsize=(12, 4))
plt.plot(np.real(sample), 'b', label='I')
plt.plot(np.imag(sample), 'r', label='Q')
plt.title("IQ-сэмплы после свёртки")
plt.xlabel("Sample index")
plt.ylabel("Amplitude")
plt.legend()
plt.grid(True)
plt.show()

symbol_period = 10
start_index = 9
num_symbols = (len(sample) - start_index) // symbol_period
eye_matrix = np.reshape(sample[start_index:start_index + num_symbols*symbol_period],
                        (num_symbols, symbol_period))

plt.figure(figsize=(10, 5))
for row in eye_matrix:
    plt.plot(np.arange(symbol_period), np.real(row), 'b', alpha=0.3)
    plt.plot(np.arange(symbol_period), np.imag(row), 'r', alpha=0.3)

plt.title("Глазковая диаграмма (Eye Diagram)")
plt.xlabel("Сэмплы внутри символа")
plt.ylabel("Амплитуда")
plt.grid(True)
plt.show()
