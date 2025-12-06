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
    # Device selection logic
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
            for i, d in enumerate(devs):
                if d.get("max_input_channels", 0) > 0:
                    print(f"[AudioSampler] PC mic found: index {i}, {d['name']}")
                    return i

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

    # ---------------------------------------------------------
    def start_stream(self, blocksize):
        self.device = self.searchForDevices()
        native_fs = self.setupDevice(self.device)
        self.rec_fs = int(native_fs)

        print(f"[FS] Using native input sample rate: {self.rec_fs} Hz")

        self.stream = sd.InputStream(
            device=self.device,
            channels=1,
            samplerate=self.rec_fs,
            blocksize=blocksize,
            dtype="float32"
        )
        self.stream.start()

        print("[Audio] Input streaming started...")

    # ---------------------------------------------------------
    def stream_blocks(self, blocksize):
        if self.stream is None:
            self.start_stream(blocksize)

        while True:
            block, _ = self.stream.read(blocksize)
            block = block.flatten()

            if len(block) > blocksize:
                block = block[:blocksize]
            elif len(block) < blocksize:
                block = np.pad(block, (0, blocksize - len(block)))

            yield block

    # ---------------------------------------------------------
    def close(self):
        if self.stream is not None:
            print("[Audio] Closing input stream...")
            self.stream.stop()
            self.stream.close()
            self.stream = None
