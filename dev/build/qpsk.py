import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lfilter

def mapper(bits):
    if len(bits) % 2 != 0:
        bits = bits[:len(bits) -1]
        
    mapping = {
        (0, 0):  1 + 1j,
        (0, 1): -1 + 1j,
        (1, 1): -1 - 1j,
        (1, 0):  1 - 1j
    }
    symbols = []
    for i in range(0, len(bits), 2):
        pair = (bits[i], bits[i + 1])
        symbols.append(mapping[pair])
    return symbols

# Параметры сигнала

N = 20
L = 10
fs = L * (N // 2) / (N // 2)
f = fs/32
t = np.linspace(0, 20, 10000)
bits = np.random.randint(0, 2, size=N).tolist()

b = np.ones(L)
a = [1]

symbols = mapper(bits)

I = np.real(symbols)
Q = np.imag(symbols)

I_upsampling = np.zeros(len(I)*L)
Q_upsampling = np.zeros(len(Q)*L)

for i in range(0, len(I)):
    I_upsampling[i*L] = I[i]
    Q_upsampling[i*L] = Q[i]

I_filtered = lfilter(b, a, I_upsampling)
Q_filtered = lfilter(b, a, Q_upsampling)

t_base = np.arange(len(I_filtered)) / L


s_rf = I_filtered * np.cos(2*np.pi*f*t_base) + Q_filtered * np.sin(2*np.pi*f*t_base)

plt.plot(t_base, s_rf)
plt.title('Модулированный радиосигнал QPSK')
plt.xlabel('Время (с)')
plt.ylabel('Амплитуда')
plt.grid(True)
plt.show()

# print(f'Исходные биты: {bits}')
# print(f'Реальная часть: {I}')
# print(f'Мнимая часть: {Q}')
# print(f'Апсемплинг I: {I_upsampling}')
# print(f'Апсемплинг Q: {Q_upsampling}')
# print(f'Формфильтер I: {I_filtered}')
# print(f'Формфильтер Q: {Q_filtered}')

plt.plot(I_filtered)
plt.title('Синфазная компонента после формирующего фильтра')
plt.xlabel('Отсчёты')
plt.ylabel('Амплитуда')
plt.grid(True)
plt.show()

plt.plot(Q_filtered)
plt.title('Квадратурная компонента после формирующего фильтра')
plt.xlabel('Отсчёты')
plt.ylabel('Амплитуда')
plt.grid(True)
plt.show()

indices = np.arange(L//2, len(I_filtered), L)
plt.scatter(Q_filtered[indices], I_filtered[indices])
plt.title('Созвездие QPSK')
plt.xlabel('Q')
plt.ylabel('I')
plt.grid(True)
plt.axis('equal')
plt.show()