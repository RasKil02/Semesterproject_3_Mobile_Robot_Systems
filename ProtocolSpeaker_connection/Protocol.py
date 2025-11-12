import numpy as np
import sounddevice as sd

class protocol:
    def __init__(self, name = "default_protocol"):
        self.name = name
        self.roomAddress = None
        self.supplyAdress = None
        self.start = '*'
        self.stop = '#'
 
    def print_command(self):
        {
            print(self.start),
            print(self.roomAddress),
            print(self.supplyAdress),
            print(self.stop)
        }
    
    def set_room_address(self, address):
        self.roomAddress = address

    def set_supply_address(self, address):
        self.supplyAdress = address

    def set_command(self):
        self.roomAddress = input("Enter room address: ")
        self.supplyAdress = input("Enter supply address: ")

    # Translate a single number to its corresponding DTMF frequencies
    def translateNumberToDTMFfreq(self, number):
        dtmf_freqs = {
            '1': (697, 1209),
            '2': (697, 1336),
            '3': (697, 1477),
            'A': (697, 1633),
            '4': (770, 1209),
            '5': (770, 1336),
            '6': (770, 1477),
            'B': (770, 1633),
            '7': (852, 1209),
            '8': (852, 1336),
            '9': (852, 1477),
            'C': (852, 1633),
            '*': (941, 1209),
            '0': (941, 1336),
            '#': (941, 1477),
            'D': (941, 1633)
        }
        return dtmf_freqs.get(number, (None, None))

    # Translate a full command string to a list of DTMF frequency pairs
    def translateCommandToDTMFfreq(self, command):
        dtmf_sequence = []
        for char in command:
            freqs = self.translateNumberToDTMFfreq(char)
            if freqs != (None, None):
                dtmf_sequence.append(freqs)
        return dtmf_sequence
    
    # Print the DTMF frequencies for the current command
    def print_DTMF_command(self):
        command = f"{self.start}{self.roomAddress}{self.supplyAdress}{self.stop}"
        dtmf_sequence = self.translateCommandToDTMFfreq(command)
        for freqs in dtmf_sequence:
            print(f"DTMF Frequencies: {freqs[0]} Hz, {freqs[1]} Hz")

    # Play the DTMF tones for the given command using numpy and sounddevice libraries
    def play_DTMF_command(self, command, duration=0.5, fs=8000):
        dtmf_sequence = self.translateCommandToDTMFfreq(command)
        print(dtmf_sequence)
        signal = np.array(dtmf_sequence)
        for freqs in signal:
                t = np.linspace(0, duration, int(fs * duration), endpoint=False)
                tone = 0.5 * (np.sin(2 * np.pi * freqs[0] * t) + np.sin(2 * np.pi * freqs[1] * t))
                sd.play(tone, fs)
                sd.wait(1)

    def compute_parity(bits: str) -> str:
        count_ones = bits.count('1')
        # Paritet: 0 hvis lige antal 1'ere, 1 hvis ulige antal 1'ere
        return '0' if count_ones % 2 == 0 else '1'

