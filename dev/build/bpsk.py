import numpy as np
import matplotlib.pyplot as plt

def mapper(bits):
    return [1 if bit == 0 else -1 for bit in bits]

# Параметры сигнала

N = 9
bits = np.random.randint(0, 2, size=N).tolist()
symbols = mapper(bits)

T = 1
f = 1/T
t = np.linspace(0, N * T, 5000, endpoint=False)
fs = 5000 / (N * T)

sig = np.zeros_like(t)

for i in range(0, len(t)):
    sym_i = int(t[i] // T)

    symbol = symbols[sym_i]

    sig[i] = np.cos(2*np.pi*f*t[i]) * symbol

plt.subplot(2, 1, 1)
plt.plot(t, sig)

plt.subplot(2, 1, 2)
plt.scatter(symbols, [0]*len(symbols), color='blue', s=100)
plt.title("созвездие BPSK")
plt.xlabel("Q")
plt.ylabel("I")
plt.grid(True)
plt.axhline(0, color='black', linewidth=1.5)
plt.axvline(0, color='black', linewidth=1.5)
plt.tight_layout()
plt.show()