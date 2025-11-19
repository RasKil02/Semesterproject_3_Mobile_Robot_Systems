import numpy as np
import sounddevice as sd


class AudioSampler:
    def __init__(self, fs):
        self.fs = fs  # Processing sample rate (normally 8000 Hz)

    def searchForDevices(self):
        try:
            devs = sd.query_devices()
            for i, d in enumerate(devs):
                if d.get('max_input_channels', 0) > 0 and "usb" in d.get('name','').lower():
                    return i
        except:
            pass
        return sd.default.device[0]

    def setupDevice(self, device):
        cur = sd.default.device
        current_out = cur[1] if isinstance(cur, (list, tuple)) else None
        sd.default.device = (device, current_out)

        info = sd.query_devices(device, "input")
        native_fs = float(info["default_samplerate"])
        print(f"[Device] name={info['name']}, default samplerate={native_fs} Hz")
        return native_fs

    def stream_blocks(self, blocksize):
        device = self.searchForDevices()
        native_fs = self.setupDevice(device)

        # Try to run the microphone directly at 8 kHz
        use_resample = False
        try:
            sd.check_input_settings(device=device, samplerate=self.fs, channels=1)
            rec_fs = self.fs
            print(f"[FS] Using telephony fs={self.fs} Hz")
        except Exception:
            # Device can't do 8 kHz → use native + resample
            rec_fs = int(native_fs)
            use_resample = True
            print(f"[FS] Device cannot run at {self.fs} Hz, using {rec_fs} Hz and resampling.")

        # Open the input stream
        with sd.InputStream(
            device=device,
            channels=1,
            samplerate=rec_fs,
            blocksize=blocksize,
            dtype="float32"
        ) as stream:

            buffer = np.zeros(0, dtype=np.float32)

            print("[Audio] Streaming started...")

            while True:
                block, _ = stream.read(blocksize)
                block = block.flatten()

                if use_resample:
                    # Resample block from rec_fs -> self.fs
                    from scipy.signal import resample_poly
                    g = np.gcd(rec_fs, self.fs)
                    up = self.fs // g
                    down = rec_fs // g
                    block = resample_poly(block, up, down).astype(np.float32)

                # If the resampling made the block the wrong size, fix it
                if len(block) > blocksize:
                    block = block[:blocksize]
                elif len(block) < blocksize:
                    pad = np.zeros(blocksize - len(block), dtype=np.float32)
                    block = np.concatenate((block, pad))
                    
                yield block
