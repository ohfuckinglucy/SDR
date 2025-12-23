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

        idx = of + Nsps + Nsps*n
        if idx >= len(samples):
            break

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

def sym_sync(samples):
    error, offset_list, offset = ted_gardner(samples)
    
    symbols = []

    for i in range(offset, len(samples), 10):
        symbols.append(samples[i])

    return symbols, error, offset_list, offset

def demapper(signal, threshold):
    bits = []

    for s in signal:
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

    # print(f'Длина бит: {len(bits)}')

    # print(f'Биты: {bits}')
    
    return bits

def trim_signal_by_energy(samples, Nsps=10, threshold_ratio=0.4):
    num_blocks = len(samples) // Nsps
    energies = []
    for i in range(num_blocks):
        start = i * Nsps
        end = start + Nsps
        block = samples[start:end]
        energy = np.mean(np.abs(block)**2)
        energies.append(energy)

    max_energy = max(energies)
    threshold = threshold_ratio * max_energy

    first_valid_block = 0
    for i in range(len(energies)):
        if energies[i] > threshold:
            first_valid_block = i
            break

    last_valid_block = 0
    for i in range(len(energies)):
        if energies[i] > threshold:
            last_valid_block = i

    start_index = first_valid_block * Nsps
    end_index = (last_valid_block + 1) * Nsps

    trimmed_samples = samples[start_index:end_index]
    return trimmed_samples

def costas_loop(samples):
    samples_fix = np.zeros_like(samples, dtype=complex)

    theta_hat = 0.0
    Kp = 0.05
    Ki = 0.005
    integrator = 0.0

    for n in range(0, len(samples)):
        r_corrected = samples[n] * np.exp(-1j * theta_hat)
        samples_fix[n] = r_corrected

        I = np.real(r_corrected)
        Q = np.imag(r_corrected)
        error = np.sign(I)*Q - np.sign(Q)*I

        integrator += error
        theta_hat += Kp*error + Ki*integrator

    return samples_fix

def create_plot(t, sig, color='r', label='', xlabel='', ylabel='', title=''):
    plt.figure()
    plt.plot(t, sig, f'{color}', label=f'{label}')
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid()
    plt.legend()

def create_dplot(t1, sig1, color1, label1, t2, sig2, color2, label2, xlabel1, ylabel2, title):
    plt.figure()
    plt.plot(t1, sig1, f'{color1}', label=f'{label1}')
    plt.plot(t2, sig2, f'{color2}', label=f'{label2}')
    plt.xlabel(xlabel1)
    plt.ylabel(ylabel2)
    plt.title(title)
    plt.grid()
    plt.legend()

def create_constellation(sig1, sig2, xlabel1, ylabel2, title):
    plt.figure()
    plt.scatter(sig1, sig2)
    plt.title(title)
    plt.xlabel(xlabel1)
    plt.ylabel(ylabel2)
    plt.axhline(0)
    plt.axvline(0)

def cfo_corr(signal, Nt = 13):
    autocor = []
    L = len(signal)

    for m in range(0, L - 2*Nt + 1):
        corr_sum = 0
        for n in range(0, Nt):
            corr_sum += signal[m + n + Nt] * np.conjugate(signal[m + n])
        autocor.append(corr_sum)
    return autocor

def cfo_estimation(symbols, s_rate=100000, Nt=13):
    autocor = cfo_corr(symbols, Nt=Nt)
    peak_index = np.argmax(np.abs(autocor))
    
    seq1 = symbols[peak_index : peak_index + Nt]
    seq2 = symbols[peak_index + Nt : peak_index + 2*Nt]
    
    a = 0
    for n in range(Nt):
        a += np.conjugate(seq1[n]) * seq2[n]
    
    phase = np.angle(a)
    
    Ts = 1 / s_rate
    f_cfo = phase / (2 * np.pi * Nt * Ts)
    
    return f_cfo, autocor

def cfo_correct(symbols, f_cfo, sample_rate=100000, n0=0):
    correct = np.zeros_like(symbols)

    n = np.arange(len(correct))

    correct = symbols * np.exp(-1j*2*np.pi*f_cfo*(n+n0)*(1/sample_rate))

    return correct

def costas_loop_bpsk(samples, Kp=0.1, Ki=0.01):
    corrected = np.zeros_like(samples, dtype=complex)
    phase_error = 0.0
    integrator = 0.0
    theta = 0.0

    for n in range(len(samples)):
        corrected[n] = samples[n] * np.exp(-1j * theta)
        
        I = np.real(corrected[n])
        Q = np.imag(corrected[n])
        phase_error = I * Q
        
        integrator += phase_error
        
        theta += Kp * phase_error + Ki * integrator
        
        theta = np.mod(theta + np.pi, 2*np.pi) - np.pi

    return corrected