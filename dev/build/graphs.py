import numpy as np
from matplotlib import pyplot as plt
import struct
print("Tx: 0\nRx: 1")
choose = int(input())

if (choose == 0):
    data = np.fromfile("tx_5.pcm", dtype=np.int16)
else:
    data = np.fromfile("rx_5.pcm", dtype=np.int16)

N = len(data) // 2

samples = []
for i in range(N):
    samples.append((data[2*i]) + 1j * (data[2*i+1]))

L = np.ones(10)
sample = np.convolve(samples, L)
sample_10 = []

for i in range(9, len(sample), 10):
    sample_10.append(sample[i])

t = np.arange(len(sample_10))

plt.figure()
plt.plot(t, np.real(sample_10), "b", label="Real")
plt.plot(t, np.imag(sample_10), "r", label="imag")
plt.legend()
plt.show()

plt.figure()
plt.scatter(np.real(sample_10), np.imag(sample_10))
plt.axhline(0)
plt.axvline(0)
plt.show()

bits = []
threshold = 500

for s in sample_10:
    if abs(s) < threshold:
        continue
    re = np.imag(s)
    im = -np.real(s)
    # re = np.real(s)
    # im = np.imag(s)
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

print(len(bits))

print(bits)