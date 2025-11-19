import argparse
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer
from SignalProc.AudioSampling import AudioSampler


def readCommand():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fs", type=int, default=44100)
    ap.add_argument("--block_ms", type=float, default=30.0)
    ap.add_argument("--hop_ms",   type=float, default=7.5)
    args = ap.parse_args()

    # --- Create detector ---
    detector = DTMFDetector(
        fs=args.fs,
        block_ms=args.block_ms,
        hop_ms=args.hop_ms,
        lowcut=620, highcut=1700, bp_order=4,
        min_db=-20, sep_db=5, dom_db=4, snr_db=8,
        twist_pos_db=+4, twist_neg_db=-8
    )

    # --- Stabilizer ---
    stabilizer = DigitStabilizer(hold_ms=20, miss_ms=20, gap_ms=55)

    # --- Audio sampler (streaming) ---
    sampler = AudioSampler(fs=args.fs)

    print("Listening for DTMF command (*#, then 5 digits)...")
    cmd = detector.stream_and_detect(stabilizer, sampler)

    print("\n--- Detected command ---")
    print(cmd if cmd else "(none)")
    return cmd


if __name__ == "__main__":
    readCommand()
