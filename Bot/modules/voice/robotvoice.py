import pyttsx

engine = pyttsx.init()

def Say(line):
    engine.say(line)
    engine.runAndWait()


Say('hello')