import numpy as np
import sounddevice as sd
import time

class Protocol:
    def __init__(self, name = "default_protocol"):
        self.name = name
        self.roomAddress = None
        self.supplyAdress = None
        self.start = '*'
        self.stop = '#'
        self.seqNr = None
        self.command = None # Command to be saved for resending

    # Set command by asking user for room and supply addresses. That number is converted to DTMF number format fx 1 -> 01
    def set_command(self):
        while True:
            while True:
                # Indtast og valider room og supply adresser
                self.roomAddress = input("Enter room address (0-3): ")
                if (0 <= int(self.roomAddress) <= 3):
                    break
                else:
                    print("Room address must be between 0 and 3. Try again: ")

            while True:
                print("Supply addresses:\n1: Supply Point 1\n2: Supply Point 2")
                self.supplyAdress = input("Enter supply address (1-2): ")

                if (1 <= int(self.supplyAdress) <= 2):
                    break
                else:
                    print("Supply address must be 1 or 2. Try again: ")
            break

        """
        Laver en kommando, hvor hvert input-tal bliver til to cifre:
        1 -> 01, 6 -> 06, 66 -> 66, 4 -> 04
        """
        room_dtmf = self.translateNumberToDTMFNumbers(self.roomAddress)
        supply_dtmf = self.translateNumberToDTMFNumbers(self.supplyAdress)

        # Saml til én samlet kommando
        self.command = f"{room_dtmf}{supply_dtmf}"
        return self.command
    
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

    # Set start bits * og # to use by playDTMFcommand
    def set_startcommand(self):
        startbit = '*'
        startbit2 = '#'
        self.startCommand = f"{startbit}{startbit2}"
        return self.startCommand
    
    def set_sequence_number(self):
        if self.seqNr is None:
            self.seqNr = '0'
        elif self.seqNr == '0':
            self.seqNr = '1'
        else:  # self.seqNr == '1'
            self.seqNr = '0'
        
        return self.seqNr

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

    # Translate a full command string to list of DTMF frequency pair.
    def translateCommandToDTMFfreq(self, command):
        dtmf_sequence = []
        for char in command:
            freqs = self.translateNumberToDTMFfreq(char)
            if freqs != (None, None):
                dtmf_sequence.append(freqs)
        return dtmf_sequence

    # Play the DTMF tones for the given command using numpy and sounddevice libraries
    def play_DTMF_command(self, command, fs, duration=0.50):

        dtmf_sequence = self.translateCommandToDTMFfreq(command)
        print(dtmf_sequence)
        signal = np.array(dtmf_sequence)
        for freqs in signal:
                t = np.linspace(0, duration, int(fs * duration), endpoint=False)
                tone = 0.5 * (np.sin(2 * np.pi * freqs[0] * t) + np.sin(2 * np.pi * freqs[1] * t))

                stereo_tone = np.column_stack((tone, tone))

                sd.play(stereo_tone, fs)
                sd.wait(1)
                time.sleep(0.28)

    # generate command with set_command. Runs checksum and gets Checksum Remainder.
    def play_dtmf_command_checksum(self, fs, command=None, resend=False):    
        # Hvis command er None, generer en ny kommando
        if command is None:
            self.set_command()  # Dette sætter self.command
            command_to_send = self.command
        else:
            # Ved genafsendelse, brug den modtagne kommando
            command_to_send = command
            # BEHOLD self.command som den er - lad være med at overskrive!

        if resend == False:
            seqNr = self.set_sequence_number()
        else:
            seqNr = self.seqNr  # Brug den tidligere sekvensnummer ved genafsendelse

        checksumString = self.calculate_crc_remainder(self.convert4BitCommandTo12BitString(command_to_send))
        print("Checksum CRC:", checksumString)

        # Konvertere 3 bit checksum til string, da play_DTMF_command tager en string som input
        checkSumDTMF = self.convert3bitToString(checksumString) 
        print("Checksum DTMF:", checkSumDTMF)

        startCommand = self.set_startcommand()  # Kald funktionen korrekt
        
        # Husk at tilføje duration parameter!
        self.play_DTMF_command(startCommand + command_to_send + checkSumDTMF + seqNr, fs, duration=0.50)


    # Converts a decimal string to a 3-bit binary string for each digit. Eksempel 01 -> 000001
    def decimal_string_to_3bit_binary_string(self, decimal_string: str) -> str:
        result = ''
        for c in decimal_string:
            num = int(c)
            if not 0 <= num <= 7:
                raise ValueError("Alle cifre skal være mellem 0 og 7 for 3-bit konvertering.")
            result += format(num, '03b')
        return result
    
    # Claculates the CRC remainder for a given a command bit string of 12 bits
    # Used on the host computer to generate the checksum digit to send to the robot.
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
    
    # Calculates a new CRC Check for a command bit string on 12 bits and a 3 bit CRC string.
    # Used on the robot to check if the received command is valid. Should give (000) if valid.
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

    # laer 4 bit string om til en 12 bit string. Eks 0001 til 000000000001
    # Command er her en 4 bit streng fra set_command 
    def convert4BitCommandTo12BitString(self, command: str) -> str:
        if len(command) % 2:
            raise ValueError("Længden skal være lige (par af cifre).")

        parts = []
        for i in range(0, len(command), 2): # Løber i gennem command og deler dem op i par
            a, b = command[i], command[i + 1]
            if a not in "01234567" or b not in "01234567": # Tjekker om hvert bit er valid
                raise ValueError("Kun 0-7 er tilladt.")
            
            parts.append(f'{format(int(a), "03b")}{format(int(b), "03b")}') # Tager et par ad gangen og laver dem om til 3-bit binary
            # og sætter dem sammen til en 6 bit string
        return "".join(parts) # Sætter 6 bit strenge om til en 12 bit string.
    
    def convert3bitToString(self, bits: str) -> str:
        if len(bits) != 3 or any(b not in '01' for b in bits):
            raise ValueError("Input must be a 3-bit binary string.")
        return str(int(bits, 2))
    
    # decode string from readcommand and use Check_CRC
    def decode_and_check_crc(self, cmd_with_startbits):
        # Udpak checksum
        print("fejl tjek, bitstring til CRC: " + cmd_with_startbits)
        checksum_digit = cmd_with_startbits[6]

        # Fjern *# og checksum → behold de 5 vigtige cifre
        print("Checksum Digit: " + checksum_digit)
        command = cmd_with_startbits[2:7]

        print("Command without startbits and seqNr: " + command)

        seqNrDigit = cmd_with_startbits[7]
        print("Sequence Number Digit: " + seqNrDigit)

        # Konverter til 3-bit binær streng
        bitstring = self.decimal_string_to_3bit_binary_string(command)
        print("Converted command to bits:", bitstring)

        # CRC check
        remainder, is_valid = self.Check_CRC(bitstring)
        print("Remainder after CRC:", remainder)
        print("Is CRC valid?", is_valid)

        # Returnér så du kan bruge det direkte i main
        return command, bitstring, is_valid, remainder, checksum_digit, seqNrDigit
