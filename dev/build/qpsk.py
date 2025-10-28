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

N = 1000
L = 10
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

iq = I_filtered + 1j*Q_filtered

pcm_i = (np.real(iq) * 32767).astype(np.int16)
pcm_q = (np.imag(iq) * 32767).astype(np.int16)

pcm_arr = np.zeros(2 * len(iq), dtype=np.int16)

pcm_arr.tofile("bits.pcm")

print(f'Исходные биты: {bits}')