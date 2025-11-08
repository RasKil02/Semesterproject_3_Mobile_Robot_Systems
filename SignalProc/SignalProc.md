All code for the signal Processing is here.


#####
File DTMFDetector.py -->  Class DTMFDetector
Explanenation of variables and thesholds:

min_db: Every peak in a block needs to be at least the value of min_db (-20)

sep_db: For every block we find the biggest and 2. biggest peak and the difference between them. 
        If the difference is under sep_db it will take it as a tone, if not, it will sort it out.
        This is good for filtering out speech.

dom_db: Looks at the difference of all peaks together. This means that medium size peaks will be sorted out.
        It takes a median of the peaks and if it is under it is sorted out.

snr_db: 