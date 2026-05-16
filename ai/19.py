import speech_recognition as sr
from deep_translator import GoogleTranslator
import requests
import json
import os

# 🔗 CHANGE THIS TO YOUR PC IP
LM_URL = "http://192.168.178.98:1234/v1/chat/completions"

SYSTEM_PROMPT = """
You are a strict information extraction system.

Extract two numbers:
- initial
- destination

Rules:
- Output ONLY in this exact format:
initial:<number>,destination:<number>
- No spaces
- No explanation
- If not found, return:
initial:0,destination:0
"""

lang = input("Enter input language (en/hi/kn etc): ").strip().lower()

recognizer = sr.Recognizer()

print("\n🎤 Voice assistant started (say 'exit' to stop)")

while True:
    try:
        with sr.Microphone(device_index=1) as source:
            print("\n🎤 Listening...")
            recognizer.adjust_for_ambient_noise(source, duration=0.5)
            audio = recognizer.listen(source, timeout=5, phrase_time_limit=8)

        # 🎤 Speech → Text
        text = recognizer.recognize_google(
            audio,
            language=f"{lang}-IN" if lang == "en" else lang
        )

        print("You said:", text)

        if text.lower() == "exit":
            print("Session ended.")
            break

        # 🌍 Translate → English
        if lang != "en":
            translated = GoogleTranslator(source=lang, target='en').translate(text)
        else:
            translated = text

        print("Translated:", translated)

        # 🤖 Send to LM Studio
        payload = {
            "model": "qwen/qwen3-vl-8b",
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": translated}
            ],
            "temperature": 0,
            "max_tokens": 50
        }

        response = requests.post(
            LM_URL,
            headers={"Content-Type": "application/json"},
            data=json.dumps(payload)
        )

        if response.status_code == 200:
            result = response.json()
            output = result["choices"][0]["message"]["content"]

            print("🤖 LLM Output:", output)

        else:
            print(f"❌ LLM Error {response.status_code}: {response.text}")

    except sr.WaitTimeoutError:
        print("⌛ Timeout")

    except sr.UnknownValueError:
        print("❌ Could not understand")

    except Exception as e:
        print("⚠️ Error:", e)