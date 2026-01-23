import numpy as np
from matplotlib import pyplot as plt

print("Tx: 0\nRx: 1")
choose = int(input())

if (choose == 0):
    data = np.fromfile("tx.pcm", dtype=np.int16)
else:
    data = np.fromfile("bin/bits3.pcm", dtype=np.int16)

N = len(data) // 2

samples = []
for i in range(N):
    samples.append(((data[2*i]) + 1j * (data[2*i+1])))

L = np.ones(10)

sample = np.convolve(samples, L)

## TED

BnTs = 0.01
Nsps = 10
zeta = np.sqrt(2) / 2
Kp = 0.002
teta = (BnTs/Nsps)/(zeta + (1/4*zeta))
epsilon = 0.001

offset = 0
p1 = 0
p2 = 0
K1 = (-4*zeta*teta)/((1+2*zeta*teta+teta**2)*Kp)
K2 = (-4*teta**2)/((1+2*zeta*teta+teta**2)*Kp)

e = []

for offset in range(0, len(sample)-22):
    n = offset
    # if (offset >= len(sample)) or (n + Nsps + Nsps >= len(sample)) or (n + Nsps >= len(sample)) or (n + Nsps//2 + Nsps >= len(sample)):
    #     break
    e_real = (np.real(sample[n + Nsps + Nsps]) - np.real(sample[n + Nsps]) * np.real(sample[n + Nsps//2 + Nsps])) # 20 - 10 * 15
    e_imag = (np.imag(sample[n + Nsps + Nsps]) - np.imag(sample[n + Nsps]) * np.imag(sample[n + Nsps//2 + Nsps]))
    e_ = e_real + e_imag

    p1 = e_*K1
    p2 = p2 + p1 + e_*K2
    
    while (p2 > 1):
        p2 = p2 - 1
    while (p2 < -1):
        p2 = p2 + 1

    offset = int(np.round(p2 * Nsps))

    e.append(e_)

# offset = (np.argmin(np.abs(e)))

print(f'aaa{offset}')

# print(f'e min = {offset} {e[offset]}')
# print(f'len samples, sample, e {len(samples)}, {len(sample)}, {len(e)}')

print(f'ofset: {offset}')
et = np.arange(len(e))

plt.figure()
plt.plot(et, e)
plt.title("е ошибки")

sample_10 = []

for i in range(offset, len(sample), 10):
    sample_10.append(sample[i])

t = np.arange(len(sample_10))

# plt.figure()
# plt.plot(t, np.real(sample_10), "b", label="Real")
# plt.plot(t, np.imag(sample_10), "r", label="Imag")
# plt.legend()

plt.figure()
plt.scatter(np.real(sample_10), np.imag(sample_10))
plt.axhline(0)
plt.axvline(0)
plt.show()

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