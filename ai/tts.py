from gtts import gTTS
import os

text = "Hello, this is Raspberry Pi text to speech working perfectly!"

tts = gTTS(text=text, lang='en')
tts.save("speech.mp3")

# Increase volume (32768 = max gain)
os.system("mpg123 -f 32768 speech.mp3")
