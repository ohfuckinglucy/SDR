import numpy as np
import matplotlib.pyplot as plt

def mapper(bits):
    if len(bits) % 2 != 0:
        print("N Должно быть кратным 2")
        exit()
        
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
bits = np.random.randint(0, 2, size=N).tolist()

symbols = mapper(bits)

T = 1
f = 1/T
t = np.linspace(0, N/2 * T, 5000, endpoint=False)
fs = 5000 / (N * T)
phase = 0

sig = np.zeros_like(t)

for i in range(len(t)):
    sym_i = int(t[i] // T)
    symbol = symbols[sym_i]
    
    I = symbol.real
    Q = symbol.imag
    
    sig[i] = I * np.cos(2 * np.pi * f * t[i]) - Q * np.sin(2 * np.pi * f * t[i])

plt.subplot(2, 1, 1)
plt.plot(t, sig)
plt.title("Сигнал")
plt.xlabel("Время, с")
plt.ylabel("Амплитуда, А")
plt.grid(True)

plt.subplot(2, 1, 2)
plt.scatter([s.real for s in symbols], [s.imag for s in symbols])
plt.title("созвездие QPSK")
plt.xlabel("I")
plt.ylabel("Q")
plt.grid(True)
plt.axhline(0, color='black', linewidth=1.5)
plt.axvline(0, color='black', linewidth=1.5)
plt.tight_layout()
plt.show()