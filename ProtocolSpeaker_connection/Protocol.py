import numpy as np
import sounddevice as sd

class Protocol:
    def __init__(self, name = "default_protocol"):
        self.name = name
        self.roomAddress = None
        self.supplyAdress = None
        self.start = '*'
        self.stop = '#'
 
    def print_command(self):
        print(self.roomAddress)
        print(self.supplyAdress)
    
    def set_room_address(self, address):
        self.roomAddress = address

    def set_supply_address(self, address):
        self.supplyAdress = address

    def set_command(self):
        self.roomAddress = input("Enter room address: ")
        self.supplyAdress = input("Enter supply address: ")

        # Konverter hver del separat
        room_dtmf = self.translateNumberToDTMFNumbers(self.roomAddress)
        supply_dtmf = self.translateNumberToDTMFNumbers(self.supplyAdress)

        # Saml til én samlet kommando
        self.command = f"{room_dtmf}{supply_dtmf}"
        return self.command


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
    
    def translateNumberToDTMFNumbers(self, command: str) -> str:
        """
        Laver en kommando, hvor hvert input-tal bliver til to cifre:
        1 -> 01, 6 -> 06, 66 -> 66, 4 -> 04
        """
        # Først tjek hele inputtet
        if not command.isdigit():
            raise ValueError("Input skal kun indeholde cifre.")

        # Derefter tjek hvert tal individuelt
        for c in command:
            number = int(c)
            if not (0 <= number <= 7):
                raise ValueError("Kun tal fra 0 til 7 er tilladt.")

        # Til sidst lav selve konverteringen
        if len(command) == 1:
            result = f"0{command}"
        elif len(command) == 2:
            result = command
        else:
            raise ValueError("Kun 1 eller 2 cifre er tilladt.")

        return result


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

    def calculate_crc_remainder(self,
     input_bitstring, poly_bitstring="1011", initial_filler='0'):
        """Calculate the CRC remainder of a string of bits using the given polynomial."""
        polynomial = list(poly_bitstring)  # Konverter polynomiet til en liste for mutabilitet
        polynomial_length = len(poly_bitstring)  # Længden af polynomiet, fx 4

        if len(input_bitstring) != 12:
            raise ValueError("Input bitstring must be 12 bits long.")

        # Append zeros to the input (length = degree of polynomial - 1)
        # Fx: "110110111011" + '0' * (4-1) = "110110111011000"
        input_padded = input_bitstring + initial_filler * (polynomial_length - 1)

        # Konverter input til liste for at kunne ændre bits under divisionen
        input_padded = list(input_padded)  # ['1', '1', '0', '1', ... '0', '0', '0']

        # Loop gennem hvert bit i den oprindelige inputstreng
        for i in range(len(input_bitstring)):
            if input_padded[i] == '1':  # Kun hvis bit er 1, udfør divisionen (XOR)
                for j in range(polynomial_length):  # Udfør XOR med polynomiets bits
                    # XOR operation: hvis bits er forskellige, bliver resultat '1', ellers '0'
                    input_padded[i + j] = str(int(input_padded[i + j] != polynomial[j]))
                    # input_padded ændres ved hver XOR operation

        # input_padded er nu den modificerede bitstreng efter divisionen

        # Resten (remainder) er de sidste (polynomial_length - 1) bits.
        remainder = ''.join(input_padded[-(polynomial_length - 1):])
        return remainder
    
    def Check_CRC(self, received_bitstring, poly_bitstring="1011"):
        polynomial = list(poly_bitstring)
        polynomial_length = len(poly_bitstring)
        input_padded = list(received_bitstring)

        for i in range(len(received_bitstring) - (polynomial_length - 1)):
            if input_padded[i] == '1':
                for j in range(polynomial_length):
                    input_padded[i + j] = str(int(input_padded[i + j] != polynomial[j]))

        remainder = ''.join(input_padded[-(polynomial_length - 1):])
        valid = remainder == '0' * (polynomial_length - 1)
        return remainder, valid


    
    def decimal_string_to_3bit_binary_string(self, decimal_string: str) -> str:
        result = ''
        for c in decimal_string:
            num = int(c)
            if not 0 <= num <= 7:
                raise ValueError("Alle cifre skal være mellem 0 og 7 for 3-bit konvertering.")
            result += format(num, '03b')
        return result


    def convertCommand(self, command: str) -> str:
        if len(command) % 2:
            raise ValueError("Længden skal være lige (par af cifre).")

        parts = []
        for i in range(0, len(command), 2):
            a, b = command[i], command[i + 1]
            if a not in "01234567" or b not in "01234567":
                raise ValueError("Kun 0-7 er tilladt.")
            parts.append(f'{format(int(a), "03b")}{format(int(b), "03b")}')
        return "".join(parts) # Laver en 12 bit samlet streng af 6 bit par
    
    def convert3bitToString(self, bits: str) -> str:
        if len(bits) != 3 or any(b not in '01' for b in bits):
            raise ValueError("Input must be a 3-bit binary string.")
        return str(int(bits, 2))

    def play_dtmf_command_checksum(self):
        command = self.set_command()
        checksumString = self.calculate_crc_remainder(self.convertCommand(command))
        print("Checksum CRC:", checksumString)
        checkSumDTMF = self.convert3bitToString(checksumString)
        print("Checksum DTMF:", checkSumDTMF)
        
        self.play_DTMF_command(command + checkSumDTMF)


    def compute_parity(bits: str) -> str:
        count_ones = bits.count('1')
        # Paritet: 0 hvis lige antal 1'ere, 1 hvis ulige antal 1'ere
        return '0' if count_ones % 2 == 0 else '1'

