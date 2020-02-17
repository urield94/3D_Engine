import simpleaudio as sa
import sys
import pathlib

this_dir = pathlib.Path(__file__).parent.absolute()

def play(path):
    wave_obj = sa.WaveObject.from_wave_file(path)
    play_obj = wave_obj.play()
    play_obj.wait_done()

def break_sound():
    play(f'{this_dir}/Break_Sound.wav')

def level_sound():
    play(f'{this_dir}/Level.wav')

def welcome_sound():
    play(f'{this_dir}/Welcome.wav')

def end_sound():
    play(f'{this_dir}/End.wav')

if __name__ == "__main__":
    try:
        sound = sys.argv[1]
        if sound == "level":
            level_sound()
        elif sound == "break":
            break_sound()
        elif sound == "welcome":
            welcome_sound()
        elif sound == "end":
            end_sound()
    except:
        pass

