import pyaudio

p = pyaudio.PyAudio()

device_index = 0  # Try 0, 1, 2... depending on which is your TONOR
device_info = p.get_device_info_by_index(device_index)

for rate in [8000, 16000, 22050, 32000, 44100, 48000]:
    try:
        if p.is_format_supported(rate,
                                 input_device=device_index,
                                 input_channels=1,
                                 input_format=pyaudio.paInt16):
            print(f"{rate} Hz supported")
    except ValueError:
        continue
