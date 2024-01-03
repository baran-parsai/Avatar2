import whisper
import soundfile as sf
import torch



model= whisper.load_model("base",device="cuda")
result = model.transcribe("sound0.wav", fp16=False)
print(result)
