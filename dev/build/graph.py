import numpy as np
from matplotlib import pyplot as plt

def ted_gardner(samples, Nsps=10):
    BnTs = 0.01
    zeta = np.sqrt(2)/2
    Kp = 1
    teta = (BnTs/Nsps)/(zeta + 1/(4*zeta))

    K1 = (-4*zeta*teta)/((1 + 2*zeta*teta + teta**2)*Kp)
    K2 = (-4*teta**2)/((1 + 2*zeta*teta + teta**2)*Kp)

    p1 = 0.0
    p2 = 0.0
    offset = 0
    n = 0
    of = 0

    offset_list = []
    error = []
    for n in range(0, len(samples)//Nsps-1):
        of = offset

        x_next = samples[of + Nsps + Nsps*n]
        x_curr = samples[of + Nsps*n]
        x_mid  = samples[of + Nsps//2 + Nsps*n]

        e_real = (np.real(x_next) - np.real(x_curr)) * np.real(x_mid)
        e_imag = (np.imag(x_next) - np.imag(x_curr)) * np.imag(x_mid)

        e = e_real + e_imag

        p1 = K1 * e
        p2 = p2 + p1 + K2 * e

        p2 %= 1

        offset = int(round(p2 * Nsps))
        offset_list.append(offset)
        error.append(e)

    return error, offset_list, offset

print("Tx: 0\nRx: 1")
choose = int(input())

if (choose == 0):
    data = np.fromfile("rx.pcm", dtype=np.int16)
else:
    data = np.fromfile("bin/lab6/rx_7.pcm", dtype=np.int16)

N = len(data) // 2

samples = []

for i in range(N):
    samples.append(((data[2*i]) + 1j * (data[2*i+1])))

samples = samples/np.max(np.real(samples))

# mask = np.abs(samples) > 0.03
# samples = samples[mask]

samples = samples[4:]

plt.plot(np.arange(len(samples)), np.real(samples), 'b', label="Real")
plt.plot(np.arange(len(samples)), np.imag(samples), 'r', label="Imag")

plt.grid()
plt.legend()
plt.show()

L = np.ones(10)

sample = np.convolve(samples, L)

sample_10 = []

error, offset_list, offset = ted_gardner(sample)

plt.plot(np.arange(len(error)), error, 'r', label="Error")
plt.plot(np.arange(len(offset_list)), offset_list, 'b', label="offset")
plt.grid()
plt.legend()
plt.show()

print(f'Найденный сдвиг: {offset}')

for i in range(offset, len(sample), 10):
    sample_10.append(sample[i])

t = np.arange(len(sample_10))

bits = []
threshold = 5

for s in sample_10:
    if abs(s) < threshold:
        continue
    re = np.real(s)
    im = np.imag(s)
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

plt.figure()
plt.plot(np.arange(len(sample)), np.real(sample), 'b', label="Реальная часть")
plt.plot(np.arange(len(sample)), np.imag(sample), 'r', label="Мнимая часть")
plt.title("Сигнал после свертки с MF")
plt.xlabel("Индекс")
plt.ylabel("Амплитуда")
plt.legend()

plt.figure()
plt.scatter(np.real(sample_10), np.imag(sample_10))
plt.axhline(0)
plt.axvline(0)
plt.show()