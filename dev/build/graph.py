import matplotlib.pyplot as plt
import numpy as np

rx = np.fromfile(f"rx.pcm", dtype=np.int16)

samples = []

for x in range(0, len(rx), 2):
    samples.append(rx[x] + 1j * rx[x+1])

ampl = np.abs(samples)
phase = np.angle(samples)
time = np.arange(len(samples))


# plot
plt.subplot(3,1,1)
plt.legend
plt.plot(time, ampl)
plt.grid(True)

plt.subplot(3,1,2)
plt.legend
plt.plot(time, phase)
plt.grid(True)


plt.subplot(3,1,3)
plt.legend
plt.plot(time, rx[0::2])
plt.plot(time, rx[1::2])
plt.grid(True)
plt.show()