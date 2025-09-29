import argparse
import csv
import datetime as dt
import struct
import sys
import time
from dataclasses import dataclass
from typing import Optional
import numpy as np
import matplotlib.pyplot as plt

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial package not found. Install with: pip install pyserial")
    sys.exit(1)

DEFAULT_BAUD = 921600

MAGIC = b"OPTA"
HDR_FMT = "<4sHHIIf f".replace(" ", "")  # magic, version,u16 ch,u32 sr,u32 n,float vref,float divider
HDR_SIZE = struct.calcsize(HDR_FMT)

@dataclass
class DeviceConfig:
    port: str
    baud: int = DEFAULT_BAUD
    timeout: float = 2.0

def find_opta_port(hint: Optional[str] = None) -> Optional[str]:
    if hint:
        return hint
    try:
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            txt = f"{p.description} {p.manufacturer}".lower()
            if any(k in txt for k in ["opta", "arduino", "stmicroelectronics"]):
                return p.device
        return ports[0].device if ports else None
    except Exception:
        return None

def gen_filename(sr_hz: int, duration_ms: int, num_ch: int) -> str:
    ts = dt.datetime.now()
    date_str = ts.strftime("%Y-%m-%d")
    time_str = ts.strftime("%H-%M-%S")  # Use dashes instead of colons for Windows compatibility
    return f"DAQ_{date_str}_{time_str}_{sr_hz}Hz_{duration_ms}ms_{num_ch}ch.csv"

def receive_exact(ser: serial.Serial, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            continue
        buf.extend(chunk)
    return bytes(buf)

def run_acquisition(cfg: DeviceConfig, duration_ms: int, out_csv: str, do_plot: bool):
    ser = serial.Serial(cfg.port, cfg.baud, timeout=cfg.timeout)
    print(f"Connected: {cfg.port} @ {cfg.baud}")

    ser.reset_input_buffer()
    time.sleep(0.05)

    cmd = f"ACQ {duration_ms}\n".encode("ascii")
    ser.write(cmd)
    ser.flush()

    # Wait for header
    hdr_bytes = receive_exact(ser, HDR_SIZE)
    magic, version, num_ch, sr_hz, num_samples, vref, divider = struct.unpack(HDR_FMT, hdr_bytes)
    if magic != MAGIC:
        # Some text may have arrived first; try to resync by reading until magic
        sync = hdr_bytes
        while True:
            b = ser.read(1)
            if not b:
                continue
            sync = (sync + b)[-len(MAGIC):]
            if sync == MAGIC:
                rest = receive_exact(ser, HDR_SIZE - len(MAGIC))
                magic, version, num_ch, sr_hz, num_samples, vref, divider = struct.unpack(HDR_FMT, MAGIC + rest)
                break

    print(f"Header: ch={num_ch}, fs={sr_hz} Hz, N={num_samples}")

    # Receive interleaved uint16 frames: num_samples * num_ch * 2 bytes
    total_bytes = num_samples * num_ch * 2
    data = receive_exact(ser, total_bytes)

    # Consume trailing newline and status, but don't block
    try:
        ser.timeout = 0.05
        ser.readline()
        ser.readline()
    except Exception:
        pass

    ser.close()

    # Interpret data
    raws = np.frombuffer(data, dtype=np.dtype('<u2')).reshape(num_samples, num_ch)
    scale = (vref / 4095.0) / max(1e-9, divider)
    volts = raws.astype(np.float32) * scale
    t = np.arange(num_samples, dtype=np.float32) / float(sr_hz)

    # Write CSV
    if not out_csv:
        out_csv = gen_filename(sr_hz, duration_ms, num_ch)
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        header = ["Time_s"] + [f"A{i}_raw" for i in range(num_ch)]
        w.writerow(header)
        for i in range(num_samples):
            sample_time = i / float(sr_hz)  # Time in seconds
            row = [f"{sample_time:.6f}"]  # 6 decimal places for microsecond precision
            row.extend(map(int, raws[i].tolist()))
            w.writerow(row)
    print(f"Wrote CSV: {out_csv}")

    # Plot if requested
    if do_plot:
        plt.figure(figsize=(10, 5))
        for ch in range(num_ch):
            plt.plot(t, volts[:, ch], label=f"A{ch}")
        plt.ylim(0.0, 10.0)
        plt.xlim(0.0, float(num_samples) / float(sr_hz))
        plt.xlabel("Time (s)")
        plt.ylabel("Voltage (V)")
        plt.title("Arduino Opta Acquisition")
        plt.legend(loc="upper right", ncol=2)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()

def main():
    ap = argparse.ArgumentParser(description="Arduino Opta Burst Acquisition Receiver")
    ap.add_argument("--port", help="Serial port (auto-detect if omitted)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--duration_ms", type=int, default=1000, help="Acquisition duration in milliseconds")
    ap.add_argument("--outfile", default="", help="Output CSV path (auto if empty)")
    ap.add_argument("--plot", action="store_true", help="Plot voltages after acquisition")
    args = ap.parse_args()

    port = find_opta_port(args.port)
    if not port:
        print("No serial port found. Specify with --port COMx (Windows) or /dev/ttyACMx.", file=sys.stderr)
        sys.exit(1)

    cfg = DeviceConfig(port=port, baud=args.baud)
    run_acquisition(cfg, max(1, args.duration_ms), args.outfile, args.plot)

if __name__ == "__main__":
    main()