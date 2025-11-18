import numpy as np
from matplotlib import pyplot as plt
import struct

data = np.fromfile("tx.pcm", dtype=np.int16)

N = len(data) // 2

real_list = []
imag_list = []

for i in range(N):
    real_list.append(data[2*i])
    imag_list.append(data[2*i + 1])

t = np.arange(len(real_list))

signal = np.array(real_list) + 1j * np.array(imag_list)

plt.subplot(2, 1, 1)
plt.plot(t, real_list, "r", label="Real")
plt.subplot(2, 1, 2)
plt.plot(t, imag_list, "b", label="Imag")
plt.legend()
plt.show()