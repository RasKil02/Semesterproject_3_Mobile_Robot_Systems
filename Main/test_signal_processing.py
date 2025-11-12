import argparse
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer
from SignalProc.Plotting import Plotting
    
def readCommand():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--fs", type=int, default=8000)
    ap.add_argument("--out", type=str, default="output.wav")
    ap.add_argument("--block_ms", type=float, default=30.0)
    ap.add_argument("--hop_ms",   type=float, default=7.5)
    args = ap.parse_args()

    detector = DTMFDetector(
        fs=args.fs,
        block_ms=args.block_ms,
        hop_ms=args.hop_ms,
        lowcut=620, highcut=1700, bp_order=4,
        min_db=-20, sep_db=5, dom_db=4, snr_db=8,
        twist_pos_db=+4, twist_neg_db=-8 
    )

    stab = DigitStabilizer(hold_ms=20, miss_ms=20, gap_ms=55)

    digits = detector.record_and_detect(args.duration, args.out, stabilizer=stab)
    print("\n--- Detected digits ---")
    print(digits if digits else "(none)")
    return digits


if __name__ == "__main__":
    readCommand()
    seg, seg_f = Plotting.open_txt_file_return_seg_segf("first_detected_block.txt")
    Plotting.plot_saved_block_tdomain(seg, seg_f)
    Plotting.plot_saved_block_fdomain(seg, seg_f)
    