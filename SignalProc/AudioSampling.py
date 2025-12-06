import numpy as np
import sounddevice as sd
import platform

class AudioSampler:
    def __init__(self, fs=44100, device_mode=None):
        self.fs = fs
        self.stream = None
        self.device = None
        self.rec_fs = None

        # If not specified, auto-detect platform
        if device_mode is None:
            system = platform.system().lower()
            if "linux" in system:
                # Raspberry Pi runs Linux, but so do PCs.
                # Detect Pi by checking CPU architecture.
                if platform.machine().startswith("arm"):
                    self.device_mode = "pi"
                else:
                    self.device_mode = "pc"
            else:
                self.device_mode = "pc"
        else:
            self.device_mode = device_mode

        print(f"[AudioSampler] Mode selected: {self.device_mode}")

    # ---------------------------------------------------------
    def searchForDevices(self):
        devs = sd.query_devices()

        # -----------------------
        # Raspberry Pi search mode
        # -----------------------
        if self.device_mode == "pi":
            for i, d in enumerate(devs):
                if d.get("max_input_channels", 0) > 0:
                    name = d.get("name", "").lower()
                    if "usb" in name or "device" in name:
                        print(f"[AudioSampler] Pi USB mic found: index {i}, {d['name']}")
                        return i

        # -----------------------
        # PC / Laptop search mode
        # -----------------------
        if self.device_mode == "pc":
            # Choose first device with input channels > 0
            try:
                devs = sd.query_devices()
                for i, d in enumerate(devs):
                    if d.get('max_input_channels', 0) > 0 and "usb" in d.get("name", "").lower():
                        return i
                
            except:
                pass
            return sd.default.device[0]

        # -----------------------
        # Fallback: use default input device
        # -----------------------
        fallback = sd.default.device[1]
        print(f"[AudioSampler] Using fallback input device index = {fallback}")
        return fallback

    # ---------------------------------------------------------
    def setupDevice(self, device):
        # Preserve OUTPUT device
        out_dev = sd.default.device[0]
        sd.default.device = (out_dev, device)

        info = sd.query_devices(device, "input")
        native_fs = float(info["default_samplerate"])
        print(f"[Device] Input device = {info['name']} @ {native_fs} Hz")
        return native_fs

    def stream_blocks(self, blocksize):
            device = self.searchForDevices()
            native_fs = self.setupDevice(device)

            # Always use the device's native sample rate
            rec_fs = int(native_fs)
            use_resample = False
            print(f"[FS] Using native sample rate: {rec_fs} Hz")

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
    
    def close(self):
        if self.stream is not None:
            print("[Audio] Closing input stream...")
            self.stream.stop()
            self.stream.close()
            self.stream = None
