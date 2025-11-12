import numpy as np
from matplotlib import pyplot as plt
import struct

<<<<<<< HEAD
data = np.fromfile("ipanema_output.pcm", dtype=np.int16)

num_samples = len(data) // 2

real_list = []
imag_list = []

for i in range(0, num_samples, 2):
    real_part = data[i]
    imag_part = data[i+1]

    real_list.append(real_part)
    imag_list.append(imag_part)

t = np.arange(len(real_list))

plt.subplot(2, 1, 1)
plt.plot(t, real_list, "r", label="Real")
plt.subplot(2, 1, 2)
plt.plot(t, imag_list, "b", label="Imag")
=======
with open("/home/plutoSDR/rubia331/SDR/dev/build/tx_out.pcm", "rb") as file:
    data = file.read()

# complex_size = 16
# num_complex = len(data) // complex_size

# symbols = []

# for i in range(num_complex):
#     offset = i * complex_size
#     real_part = struct.unpack('d', data[offset:offset+8])[0]
#     imag_part = struct.unpack('d', data[offset+8:offset+16])[0]

#     symbols.append(complex(real_part, imag_part))

# t = np.arange(len(symbols))

real_list = []
imag_list = []

for i in range(0, len(data), 2):
    real_list.append(data[i])
    imag_list.append(data[i+1])

t = np.arange(len(real_list))

plt.plot(t, real_list, "r", label="Real")
plt.plot(t, imag_list, "b", label="Imag")
plt.xlabel("Sample Index")
plt.ylabel("Amplitude")
>>>>>>> a4c770b (Не получилось передать биты)
plt.legend()
plt.show()