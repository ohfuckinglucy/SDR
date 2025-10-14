import numpy as np
import librosa
from pydub import AudioSegment
import scipy.signal

with open("rx_samples.pcm", 'rb') as f:
    pcm_data = f.read()


# pcm_data.tofile("ipanema.pcm")

audio = AudioSegment(
    data=pcm_data,
    sample_width=2,      # 2 байта = 16 бит
    frame_rate=44100,    # частота дискретизации
    channels=1           # моно
)

audio.export("ipanemaEXP.mp3", format="mp3", bitrate="192k")