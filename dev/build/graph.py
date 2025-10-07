import numpy as np
import matplotlib.pyplot as plt

# try:
#     with open("/home/plutoSDR/Rubtsov/lab2/dev/build/rx_samples.bin", "rb") as file:
#         data_iq_str = file.read()
# except:
#     print("Ошибка чтения")

data_iq = np.fromfile("/home/plutoSDR/Rubtsov/lab2/dev/build/tx_samples.pcm", dtype=np.int16)
data_iq2 = np.fromfile("/home/plutoSDR/Rubtsov/lab2/dev/build/rx_samples.pcm", dtype=np.int16)

if len(data_iq) % 2 != 0:
    print("длина данных нечетная, последний сэмпл будет отброшен")
    data_iq = data_iq[:len(data_iq) - 1]

# # plt.plot(data_iq)
# # plt.show()



iq_samples = data_iq[0::2] + 1j * data_iq[1::2]

# plt.subplot(2,1,1)

# plt.plot(data_iq[0::2], "m")
# plt.subplot(2,1,2)

# plt.plot(data_iq[1::2], "g")

# plt.show()

amplitude = np.abs(iq_samples)

phase = np.angle(iq_samples)
time = np.arange(len(iq_samples))

plt.figure(figsize=(12, 6))

plt.subplot(2,1,1)
plt.plot(time, amplitude, "r")
plt.title("амплитуда Tx")
plt.xlabel("семпл индкс")
plt.ylabel("амплитуда")
plt.grid(True)

# plt.subplot(2,1,2)
# plt.plot(time, phase, "m")
# plt.title("фаза")
# plt.xlabel("семпл индекс")
# plt.ylabel("фаза (радианы)")
# plt.grid(True)

# plt.tight_layout()
# plt.show()





if len(data_iq) % 2 != 0:
    print("длина данных нечетная, последний сэмпл будет отброшен")
    data_iq = data_iq[:len(data_iq) - 1]

# # plt.plot(data_iq)
# # plt.show()



iq_samples = data_iq2[0::2] + 1j * data_iq2[1::2]

# plt.subplot(2,1,1)

# plt.plot(data_iq[0::2], "m")
# plt.subplot(2,1,2)

# plt.plot(data_iq[1::2], "g")

# plt.show()

amplitude = np.abs(iq_samples)

phase = np.angle(iq_samples)
time = np.arange(len(iq_samples))

# plt.figure(figsize=(12, 6))

plt.subplot(2,1,2)
plt.plot(time, amplitude, "r")
plt.title("амплитуда Rx")
plt.xlabel("семпл индкс")
plt.ylabel("амплитуда")
plt.grid(True)

# plt.subplot(2,1,2)
# plt.plot(time, phase, "m")
# plt.title("фаза")
# plt.xlabel("семпл индекс")
# plt.ylabel("фаза (радианы)")
# plt.grid(True)

plt.tight_layout()
plt.show()