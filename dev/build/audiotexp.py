import numpy as np
from scipy.io.wavfile import write

# Read as int16 real samples (not complex!)
samples = np.fromfile("rx_samples.pcm", dtype=np.int16)

# Optional: decimate from 1e6 to 48e3
from scipy.signal import decimate
audio_48k = decimate(samples, 1_000_000 // 48_000).astype(np.int16)

write("output.wav", 48000, audio_48k)