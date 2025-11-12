import numpy as np
import matplotlib.pyplot as plt
import sounddevice as sd
from scipy.io import wavfile
from scipy.signal import resample_poly

class AudioSampler:
    def __init__(self, duration, fs, output):
        self.duration = duration
        self.fs = fs
        self.output = output
        self.audio = None
    
    def searchForDevices(self):
        # Find første input-device med "usb" i navnet
        try:
            devs = sd.query_devices()
            for i, d in enumerate(devs):
                if d.get('max_input_channels', 0) > 0 and 'usb' in d.get('name','').lower():
                    return i
        except Exception:
            pass
        # Fallback: brug default input
        return sd.default.device[0]
    
    def setupDevice(self, device):
    # Behold eksisterende output-device (sæt ikke None)
        cur = sd.default.device
        current_out = cur[1] if isinstance(cur, (list, tuple)) else None
        sd.default.device = (device, current_out)  # <-- denne linje er fixet

        info = sd.query_devices(device, 'input')
        native_fs = float(info['default_samplerate'])
        print(f"[Device] name={info['name']}, default_samplerate={native_fs} Hz")
        return native_fs
        
    def record_audio(self, device=None):
        # Find og sæt device
        device = self.searchForDevices()
        native_fs = self.setupDevice(device)

        # Vælg strategi:
        USE_TELEPHONY_FS = True  # True = A (8 kHz), False = B (native fs)

        if USE_TELEPHONY_FS:
            target_fs = 8000.0
            # Prøv at tjekke om device kan køre target_fs
            try:
                sd.check_input_settings(device=device, samplerate=target_fs, channels=1, dtype='float32')
                self.fs = int(target_fs)
                print(f"[FS] Using telephony fs={self.fs} Hz")
                rec_fs = self.fs
            except Exception as e:
                # fallback: optag i native og resampl bagefter
                self.fs = int(target_fs)  # behold “behandlings-fs” som 8 kHz
                rec_fs = int(native_fs)
                print(f"[FS] Device cannot do {target_fs} Hz directly; recording at {rec_fs} Hz and resampling to {self.fs} Hz. ({e})")
        else:
            # Brug native hele vejen
            self.fs = int(native_fs)
            rec_fs = self.fs
            print(f"[FS] Using device native fs={self.fs} Hz")

        # Optag med rec_fs (den reelt brugte samplerate)
        print(f"Recording from device {device} for {self.duration} s at {rec_fs} Hz...")
        sd.default.samplerate = rec_fs
        self.audio = sd.rec(int(self.duration * rec_fs),
                            samplerate=rec_fs, channels=1, dtype='float32')
        sd.wait()

        # niveau-diagnostik
        a = self.audio.squeeze().astype(float)
        peak = float(np.max(np.abs(a)))
        rms  = float(np.sqrt(np.mean(a*a)))
        print(f"[Audio] peak={peak:.6f}, rms={rms:.6f}")
        print("Recording complete.")

        # Resampl hvis nødvendigt
        a = self.audio.flatten().astype(np.float32)
        if rec_fs != self.fs:
            from scipy.signal import resample_poly
            g = np.gcd(int(rec_fs), int(self.fs))
            up, down = self.fs // g, int(rec_fs) // g
            print(f"[Resample] {rec_fs} Hz -> {self.fs} Hz (up={up}, down={down})")
            a = resample_poly(a, up, down).astype(np.float32)
        self.audio = a
        return self.audio
    
    def save_audio(self):
        import numpy as np
        from scipy.io import wavfile
        if self.audio is None:
            raise RuntimeError("Call record_audio() first.")
        a = self.audio.squeeze().astype('float32')
        peak = float(np.max(np.abs(a))) or 1.0
        a = a / peak   # KUN for at kunne høre den ved afspilning
        wavfile.write(self.output, self.fs, (a*32767).astype(np.int16))
        print(f"Audio saved to {self.output} (normalized)")
