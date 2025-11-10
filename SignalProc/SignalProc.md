Short explanation of the signal processing code bellow:

### Generel overview ###
The signal processing code consists of 5 classes:
1. AudioSampling
        - Record and save 
2. BandPassFilter
        - Filter
3. Goertzel
        - Finds the tone by measuring the energy
4. DigitStabalizer
        - Checks if the tone is stable over time.
5. DTMFDetector (This is the big one - main code for DSP)
        - Puts it all together



### DTMFDetector class ###
File DTMFDetector.py -->  Class DTMFDetector

--- Generel explenation of class ---
The DTMFDetector class records or analyzes audio to detect which DTMF digits (telephone keypad tones) are present.
It splits the audio into short time blocks, filters the signal, and measures the energy at the eight DTMF frequencies using the Goertzel algorithm.
It then decides which two frequencies (one low, one high) are dominant in each block and checks several conditions — like loudness, separation, SNR, and twist — to verify if they form a valid DTMF tone.
Each valid detection is sent to a DigitStabilizer to confirm the tone is stable over time before it’s accepted as a real digit.
Finally, the detector returns the sequence of recognized digits as a string (e.g., "123#").


--- Explanenation of variables and thesholds ---

min_db: Every peak in a block needs to be at least the value of min_db (-20)

sep_db: For every block we find the biggest and 2. biggest peak and the difference between them. 
        If the difference is under sep_db it will take it as a tone, if not, it will sort it out.
        This is good for filtering out speech.

dom_db: Looks at the difference of all peaks together. This means that medium size peaks will be sorted out.
        It takes a median of the peaks and if it is under it is sorted out.

snr_db: Signal-to-noise ratio (SNR) is a measure of how strong a signal (DTMF tone) is compared to the background noise.
        It is used to make sure that the actual tone is significantly louder than the background noise in that frequency range.
        It is calculated as snr_db = 10*log_10(Power_of_signal / Power_of_Noise).
        So if we played music with the same amount of db as our DTMF tone, we wouldt be able to detect the right DTMF tone... (This needs to be fixed)


twist_pos_db: Is the difference in loudness between the low and the high frequencies.
              In a DTMF tone there is to tones at the same time! One high frequency (1209, 1336, 1447, 1633) and one low frecuency (697, 770, 852, 941).
              Sometimes in the real world one of those tones might be a little higher than the other. That is called the twist!
              This is important! Because if one tone os much higher than the other, the weaker one can be buried in noise and the detector might think it is a different tone. That is why the detector only allows frecuencies that are balanced. 
              So positive twist in db is the maximum allowed positive difference (low > high).

twist_neg_db: Same as above! Just the maximum allowed negative difference (low < high).