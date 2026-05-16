import speech_recognition as sr
from gtts import gTTS
from deep_translator import GoogleTranslator
import os
import time

# Step 1: Ask language
lang = input("Enter input language (en/hi/kn etc): ").strip().lower()

recognizer = sr.Recognizer()

print("\n🎤 Voice assistant started (say 'exit' to stop)")

while True:
    try:
        # Step 2: Initialize mic INSIDE loop properly
        with sr.Microphone(device_index=1) as source:   # 👈 pulse mic
            print("\n🎤 Listening...")
            recognizer.adjust_for_ambient_noise(source, duration=0.5)
            audio = recognizer.listen(source, timeout=5, phrase_time_limit=8)

        # Step 3: Speech → Text
        text = recognizer.recognize_google(audio, language=f"{lang}-IN" if lang=="en" else lang)
        print("You said:", text)

        # Exit condition
        if text.lower() == "exit":
            print("Session ended.")
            break

        # Step 4: Translate if needed
        if lang != "en":
            translated = GoogleTranslator(source=lang, target='en').translate(text)
            print("Translated:", translated)
        else:
            translated = text

        # Step 5: Text → Speech
        tts = gTTS(text=translated, lang='en')
        tts.save("output.mp3")

        # Step 6: Play audio (better compatibility)
        os.system("mpg321 output.mp3")

        time.sleep(0.5)

    except sr.WaitTimeoutError:
        print("⌛ Listening timed out... try speaking again")

    except sr.UnknownValueError:
        print("❌ Could not understand audio")

    except sr.RequestError as e:
        print(f"❌ API error: {e}")

    except Exception as e:
        print("⚠️ Error:", e)