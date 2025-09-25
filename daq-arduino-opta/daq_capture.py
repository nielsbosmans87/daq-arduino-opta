import argparse
import csv
import datetime as dt
import sys
import time
from dataclasses import dataclass
from typing import Optional, List

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial package not found. Install with: pip install pyserial")
    sys.exit(1)

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

DEFAULT_BAUD = 921600

@dataclass
class DeviceConfig:
    port: str
    baud: int = DEFAULT_BAUD
    timeout: float = 0.2  # seconds

def find_opta_port(hint: Optional[str] = None) -> Optional[str]:
    if hint:
        return hint
    
    # Use serial.tools.list_ports to find available ports
    try:
        ports = serial.tools.list_ports.comports()
        if ports:
            # Look for Arduino Opta specifically by manufacturer/description
            for port in ports:
                desc = f"{port.description} {port.manufacturer}".lower()
                if any(keyword in desc for keyword in ['arduino', 'stmicroelectronics', 'opta']):
                    return port.device
            # If no specific match, return the first available port
            return ports[0].device
    except Exception:
        pass
    
    # Fallback: return likely port names
    import platform
    system = platform.system().lower()
    if 'windows' in system:
        return 'COM3'  # Common Arduino port on Windows
    else:
        return '/dev/ttyACM0'  # Common Arduino port on Linux/Mac

def gen_filename(prefix: str = "ADC_Data") -> str:
    ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{ts}.csv"

def parse_data_line(line: str):
    # Lines start with "DATA," or "PLOT," or "STAT,"
    if line.startswith("DATA,"):
        parts = line.strip().split(',')
        # Expected: DATA,Timestamp_ms, A0_raw, A0_volts, ..., A7_raw, A7_volts
        # len = 1 + 1 + (NUM_CH * (1 or 2))
        try:
            t_ms = float(parts[1])
            values = parts[2:]
            # auto-detect whether volts present (pairs)
            ch = 8
            have_volts = (len(values) == ch * 2)
            raws = []
            volts = []
            if have_volts:
                for i in range(ch):
                    raws.append(int(values[2*i]))
                    volts.append(float(values[2*i+1]))
            else:
                for i in range(ch):
                    raws.append(int(values[i]))
            return ("DATA", t_ms, raws, volts if have_volts else None)
        except Exception:
            return None
    elif line.startswith("PLOT,"):
        # For live plot, we only need the last four channel raw/volts quick view; can be ignored for CSV
        return ("PLOT", line.strip(), None, None)
    elif line.startswith("STAT,"):
        return ("STAT", line.strip(), None, None)
    return None

def acquire(config: DeviceConfig, duration_s: Optional[float], out_csv: str,
            live_plot: bool, no_header: bool):
    ser = serial.Serial(config.port, config.baud, timeout=config.timeout)
    print(f"Connected: {config.port} @ {config.baud}")

    # Clear input
    ser.reset_input_buffer()
    time.sleep(0.1)

    # Command start
    ser.write(b"s\n")
    ser.flush()

    # CSV setup
    ch = 8
    if not out_csv:
        out_csv = gen_filename()
    f = open(out_csv, "w", newline="")
    writer = csv.writer(f)

    if not no_header:
        header = ["Timestamp_ms"]
        for i in range(ch):
            header.append(f"A{i}_raw")
            header.append(f"A{i}_volts")
        writer.writerow(header)

    # Live plot setup
    fig, ax = (None, None)
    lines = []
    window_sec = 5.0
    xs = []
    ys = [ [] for _ in range(4) ]  # show 4 channels
    last_plot_update = time.time()

    if live_plot:
        plt.style.use("seaborn-v0_8")
        fig, ax = plt.subplots(1, 1, figsize=(10, 4))
        for i in range(4):
            ln, = ax.plot([], [], label=f"A{i}")
            lines.append(ln)
        ax.set_xlim(0, window_sec)
        ax.set_ylim(0, 10.5)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Volts")
        ax.legend(loc="upper right")
        plt.show(block=False)  # Show the plot window without blocking

    start_t = time.time()
    first_timestamp = None  # Track first timestamp for normalization
    bytes_buf = bytearray()
    total_rows = 0

    try:
        while True:
            if duration_s is not None and (time.time() - start_t) >= duration_s:
                break

            chunk = ser.readline()  # reads until newline or timeout
            if not chunk:
                continue

            try:
                line = chunk.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue

            parsed = parse_data_line(line)
            if not parsed:
                continue

            kind, payload, raws, volts = parsed
            if kind == "DATA":
                t_ms_raw = payload
                
                # Normalize timestamp to start at 0
                if first_timestamp is None:
                    first_timestamp = t_ms_raw
                t_ms = t_ms_raw - first_timestamp
                
                if volts is None:
                    # compute volts if not provided
                    volts = [raw * (3.3/4095.0) / 0.3034 for raw in raws]
                row = [f"{t_ms:.3f}"]
                for i in range(8):
                    row.append(str(raws[i]))
                    row.append(f"{volts[i]:.4f}")
                writer.writerow(row)
                total_rows += 1

                # live plot update throttled
                if live_plot:
                    t_s = (t_ms / 1000.0)
                    xs.append(t_s)
                    for i in range(4):
                        ys[i].append(volts[i])
                    # trim window
                    while xs and (t_s - xs[0]) > window_sec:
                        xs.pop(0)
                        for i in range(4):
                            ys[i].pop(0)
                    now = time.time()
                    if now - last_plot_update > 0.05:
                        for i in range(4):
                            lines[i].set_data(xs, ys[i])
                        if xs:
                            ax.set_xlim(max(0, xs[-1] - window_sec), xs[-1])
                        ax.figure.canvas.draw()
                        ax.figure.canvas.flush_events()
                        last_plot_update = now

            elif kind == "STAT":
                # Print compact status
                print(payload)

            # PLOT lines are ignored for CSV

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # Stop device
        try:
            ser.write(b"x\n")
            ser.flush()
        except Exception:
            pass

        f.flush()
        f.close()
        ser.close()
        print(f"Wrote CSV: {out_csv} ({total_rows} rows)")

        if live_plot and fig is not None:
            plt.close(fig)

def main():
    ap = argparse.ArgumentParser(description="Arduino Opta ADC Data Capture")
    ap.add_argument("--port", help="Serial port (auto-detect if omitted)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--duration", type=float, default=10.0, help="Seconds; omit/negative for continuous")
    ap.add_argument("--outfile", default="", help="Output CSV path (auto if empty)")
    ap.add_argument("--plot", action="store_true", help="Enable live plot (first 4 channels)")
    ap.add_argument("--no-header", action="store_true", help="Do not write CSV header")
    args = ap.parse_args()

    port = find_opta_port(args.port)
    if not port:
      print("No serial port found. Specify with --port COMx (Windows) or /dev/ttyACMx.", file=sys.stderr)
      sys.exit(1)

    duration_s = None if (args.duration is None or args.duration < 0) else args.duration
    cfg = DeviceConfig(port=port, baud=args.baud)
    acquire(cfg, duration_s, args.outfile, args.plot, args.no_header)

if __name__ == "__main__":
    main()