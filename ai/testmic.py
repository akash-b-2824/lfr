import speech_recognition as sr

for i, name in enumerate(sr.Microphone.list_microphone_names()):
    print(i, name)
