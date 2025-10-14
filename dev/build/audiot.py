import numpy as np
import librosa
from pydub import AudioSegment
import scipy.signal

y, sr = librosa.load("ipanema.mp3", sr=44100, mono=True)

pcm_data = (y * 32767).astype(np.int16)

pcm_data.tofile("ipanema.pcm")

audio = AudioSegment(
    data=pcm_data.tobytes(),
    sample_width=2,      # 2 байта = 16 бит
    frame_rate=44100,    # частота дискретизации
    channels=1           # моно
)