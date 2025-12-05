import numpy as np
import sounddevice as sd

class AudioSampler:
    def __init__(self, fs=44100):
        self.fs = fs
        self.stream = None
        self.device = None
        self.rec_fs = None

    def searchForDevices(self):
        try:
            devs = sd.query_devices()
            for i, d in enumerate(devs):
                if d.get('max_input_channels', 0) > 0 and "usb" in d.get('name','').lower():
                    return i
        except:
            pass
        return sd.default.device[1]  # input index

    def setupDevice(self, device):
        # keep output device unchanged
        cur = sd.default.device
        current_out = cur[0]

        sd.default.device = (current_out, device)

        info = sd.query_devices(device, "input")
        native_fs = float(info["default_samplerate"])
        print(f"[Device] name={info['name']}, default samplerate={native_fs} Hz")
        return native_fs

    def start_stream(self, blocksize):
        self.device = self.searchForDevices()
        native_fs = self.setupDevice(self.device)
        self.rec_fs = int(native_fs)

        print(f"[FS] Using native sample rate: {self.rec_fs} Hz")

        # Create stream, but DO NOT use a `with` block
        self.stream = sd.InputStream(
            device=self.device,
            channels=1,
            samplerate=self.rec_fs,
            blocksize=blocksize,
            dtype="float32"
        )
        self.stream.start()

        print("[Audio] Streaming started...")

    def stream_blocks(self, blocksize):
        if self.stream is None:
            self.start_stream(blocksize)

        while True:
            block, _ = self.stream.read(blocksize)
            block = block.flatten()

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
