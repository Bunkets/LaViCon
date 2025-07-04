import pyaudio
import json
from vosk import Model, KaldiRecognizer
model = Model(model_path="/home/ren/vocar_ws/vosk-api/python/example/vosk-model-small-en-us-0.15" )
rec = KaldiRecognizer(model, 16000)
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4096)
stream.start_stream()
try:
    while stream.is_active():
        data = stream.read(4096)
        if rec.AcceptWaveform(data):
            print(json.loads(rec.Result()))
        else:
            res = json.loads(rec.PartialResult())
            if res["partial"]:
                print('partial ' + res['partial'])
except KeyboardInterrupt:
    print("Stop recording...")