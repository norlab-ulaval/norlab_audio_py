import sounddevice as sd
import numpy as np
import wave

# Parameters
device = 0  # Adjust according to your device index for hw:1,0
sample_rate = 44100  # Sample rate in Hz
duration = 10  # Duration of recording in seconds
filename = f'/home/robot/output{device}.wav'  # Output file name

# Record audio
print("Recording...")
audio_data = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, device=device, dtype='float32')
sd.wait()  # Wait until recording is finished
print("Recording finished.")

# Save as WAV file
with wave.open(filename, 'wb') as wf:
    wf.setnchannels(1)  # Mono
    wf.setsampwidth(2)  # 16-bit PCM
    wf.setframerate(sample_rate)
    wf.writeframes(np.int16(audio_data * 32767).tobytes())  # Convert to int16 for WAV format

print(f"Audio saved as {filename}.")
