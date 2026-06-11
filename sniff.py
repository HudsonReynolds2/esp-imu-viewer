#!/usr/bin/env python3
"""
sniff.py - show exactly what the ESP32 is sending on the serial port.

Usage:
    pip install pyserial
    python sniff.py COM4

Close the DFRobot visualizer and any idf.py monitor first (only one program
can hold the port). Press the reset button on the XIAO after starting this so
you also capture the boot output.

It prints each received line twice:
  RAW : with control bytes shown as \r \n etc, so we can see line endings
  REPR: Python repr() of the chunk, the ground truth of every byte
"""
import sys
import serial   # from pyserial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD = 115200

def visible(b: bytes) -> str:
    out = []
    for ch in b:
        if ch == 0x0d:
            out.append("\\r")
        elif ch == 0x0a:
            out.append("\\n")
        elif ch == 0x09:
            out.append("\\t")
        elif 32 <= ch < 127:
            out.append(chr(ch))
        else:
            out.append(f"\\x{ch:02x}")
    return "".join(out)

def main():
    print(f"Opening {PORT} @ {BAUD}. Ctrl-C to stop. Reset the board now.\n")
    with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
        buf = bytearray()
        try:
            while True:
                chunk = ser.read(256)
                if not chunk:
                    continue
                buf.extend(chunk)
                # split on \n but keep showing the raw bytes including \r
                while b"\n" in buf:
                    idx = buf.index(b"\n")
                    line = bytes(buf[:idx + 1])
                    del buf[:idx + 1]
                    print(f"RAW : {visible(line)}")
                    print(f"REPR: {line!r}")
        except KeyboardInterrupt:
            if buf:
                print(f"RAW : {visible(bytes(buf))}   (no trailing newline)")
                print(f"REPR: {bytes(buf)!r}")
            print("\nstopped.")

if __name__ == "__main__":
    main()
