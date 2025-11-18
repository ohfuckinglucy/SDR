import numpy as np
from matplotlib import pyplot as plt
import struct
print("Tx: 0\nRx: 1")
choose = int(input())

if (choose == 0):
    data = np.fromfile("tx.pcm", dtype=np.int16)
else:
    data = np.fromfile("rx.pcm", dtype=np.int16)

N = len(data) // 2

samples = []

for i in range(N):
    samples.append((data[2*i]/np.max(data)) + 1j * (data[2*i+1]/np.max(data)))

L = np.ones(10)
sample = np.convolve(samples, L)
sample_10 = []

for i in range(9, len(sample), 10):
    sample_10.append(sample[i])



t = np.arange(len(sample_10))

plt.figure()
plt.plot(t, np.imag(sample_10), "r", label="Real")
plt.legend()
plt.show()

plt.figure()
plt.scatter(np.real(sample_10), np.imag(sample_10))
plt.axhline(0)
plt.axvline(0)
plt.show()

bits = []

for i in range(0, len(sample_10)):
    if np.real(sample_10[i]) > 5:
        if np.imag(sample_10[i]) > 5:
            bits.append(0)
            bits.append(0)
        elif np.imag(sample_10[i]) < 5:
            bits.append(1)
            bits.append(1)
    elif np.real(sample_10[i] < -5):
        if np.imag(sample_10[i]) > 5:
            bits.append(0)
            bits.append(1)
        elif np.imag(sample_10[i]) < -5:
            bits.append(1)
            bits.append(0)

print(bits)